#include <cleany_moveit_collision/cad_linkage.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace cleany
{
namespace
{
Eigen::Vector2d turn(const Eigen::Vector2d& p, double angle)
{
  const double c = std::cos(angle), s = std::sin(angle);
  return {c*p.x()-s*p.y(), s*p.x()+c*p.y()};
}

double angle(const Eigen::Vector2d& a, const Eigen::Vector2d& b)
{
  return std::atan2(a.x()*b.y()-a.y()*b.x(), a.dot(b));
}

bool circle(const Eigen::Vector2d& a, const Eigen::Vector2d& b, double ra, double rb,
            int branch, double epsilon, Eigen::Vector2d& result)
{
  const auto delta = (b-a).eval();
  const double distance = delta.norm();
  if (!std::isfinite(distance) || distance <= epsilon) return false;
  const double along = (ra*ra-rb*rb+distance*distance)/(2.*distance);
  const double height = ra*ra-along*along;
  if (!std::isfinite(height) || height <= epsilon*epsilon) return false;
  const auto unit = (delta/distance).eval();
  result = a+along*unit+branch*std::sqrt(height)*Eigen::Vector2d(-unit.y(),unit.x());
  return result.allFinite();
}
}  // namespace

CadLinkage::CadLinkage(const std::string& json, const moveit::core::RobotModel& model)
{
  const auto config = YAML::Load(json);
  if (config["schema"].as<std::string>() != "cleany.calibrated_cad/v1")
    throw std::invalid_argument("Unknown calibrated CAD linkage schema");
  const auto& variables = model.getVariableNames();
  auto require_variable = [&](const std::string& name) {
    if (std::find(variables.begin(),variables.end(),name) == variables.end())
      throw std::invalid_argument("CAD linkage variable is missing: "+name);
  };
  if (!config["grippers"].IsSequence() || config["grippers"].size() != 2)
    throw std::invalid_argument("Both calibrated CAD grippers are required");
  for (const auto& group : config["grippers"])
  {
    Gripper g;
    g.source = group["source_joint"].as<std::string>();
    require_variable(g.source);
    g.scale = group["scale"].as<double>();
    g.offset = group["offset_rad"].as<double>();
    g.epsilon = group["linkage"]["numerical_tolerance_m"].as<double>();
    if (!std::isfinite(g.scale+g.offset+g.epsilon) || g.scale == 0. || g.epsilon <= 0.)
      throw std::invalid_argument("Invalid calibrated CAD cam conversion");
    for (const auto& entry : group["linkage"]["fingers"])
    {
      const auto spec = entry.second;
      auto point = [&](const char* key) {
        const auto p = spec["points_m"][key];
        if (!p.IsSequence() || p.size() != 2)
          throw std::invalid_argument("CAD pin coordinates must be two-dimensional");
        Eigen::Vector2d value(p[0].as<double>(),p[1].as<double>());
        if (!value.allFinite()) throw std::invalid_argument("Nonfinite CAD pin");
        return value;
      };
      Finger f;
      f.a=point("A"); f.b=point("B"); f.c=point("C");
      f.d=point("D"); f.e=point("E"); f.p=point("P");
      const auto lengths=spec["lengths_m"];
      f.ad=lengths["AD"].as<double>(); f.pd=lengths["PD"].as<double>();
      f.ac=lengths["AC"].as<double>(); f.be=lengths["BE"].as<double>(); f.ce=lengths["CE"].as<double>();
      f.drive_branch=spec["branch_drive"].as<int>(); f.distal_branch=spec["branch_distal"].as<int>();
      for (double length : {f.ad,f.pd,f.ac,f.be,f.ce})
        if (!std::isfinite(length) || length <= 0.) throw std::invalid_argument("Invalid CAD pin length");
      if (std::abs(f.drive_branch) != 1 || std::abs(f.distal_branch) != 1)
        throw std::invalid_argument("Invalid saved CAD assembly branch");
      std::size_t i=0;
      for (const char* key : {"internal","external","distal","driving"})
      {
        f.names[i]=spec["joints"][key].as<std::string>();
        require_variable(f.names[i++]);
      }
      g.fingers.push_back(std::move(f));
    }
    if (g.fingers.size() != 2) throw std::invalid_argument("A CAD gripper must have two fingers");
    groups_.push_back(std::move(g));
  }
}

bool CadLinkage::resolve(moveit::core::RobotState& state) const
{
  // These are the existing Python solve_linkage equations, including its
  // saved-assembly branch and rejection of unresolved toggles.
  std::vector<std::pair<std::string,double>> updates;
  for (const auto& group : groups_)
  {
    const double cam=group.scale*state.getVariablePosition(group.source)+group.offset;
    if (!std::isfinite(cam)) return false;
    for (const auto& f : group.fingers)
    {
      const auto p=turn(f.p,cam);
      Eigen::Vector2d d,e;
      if (!circle(f.a,p,f.ad,f.pd,f.drive_branch,group.epsilon,d)) return false;
      const double internal=angle(f.d-f.a,d-f.a);
      const auto c=(f.a+turn(f.c-f.a,internal)).eval();
      if (!circle(c,f.b,f.ce,f.be,f.distal_branch,group.epsilon,e)) return false;
      const double external=angle(f.e-f.b,e-f.b);
      const double distal=angle(f.e-f.c,e-c)-internal;
      const double driving=angle(f.d-f.p,d-p)-cam;
      const std::array<double,4> values{internal,external,distal,driving};
      for (std::size_t i=0;i<values.size();++i)
      {
        if (!std::isfinite(values[i])) return false;
        updates.emplace_back(f.names[i],values[i]);
      }
    }
  }
  for (const auto& update : updates) state.setVariablePosition(update.first,update.second);
  state.update();
  return true;
}
}  // namespace cleany
