#include "octomap_cuda.hpp"
#include <cleany_interfaces/srv/check_state_validity_batch.hpp>
#include <cleany_moveit_collision/calibrated_cad_collision.hpp>
#include <moveit/move_group/move_group_capability.h>
#include <moveit/move_group/move_group_context.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/shape/box.h>
#include <pluginlib/class_list_macros.hpp>
#include <octomap/OcTree.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <map>
#include <stdexcept>
#include <set>
#include <sstream>

namespace cleany
{
namespace
{
using Clock=std::chrono::steady_clock;
using Vec=Eigen::Vector3d;
using Robot=moveit::core::RobotState;
using Scene=planning_scene::PlanningScene;
using Matrix=collision_detection::AllowedCollisionMatrix;
using Service=cleany_interfaces::srv::CheckStateValidityBatch;

class ModelGeometry : public cleany::CADInspectionCollisionEnv
{
 public:
  explicit ModelGeometry(const Scene& scene):CADInspectionCollisionEnv(scene.getRobotModel(),0.,1.)
  {
    for(const auto* link:scene.getRobotModel()->getLinkModelsWithCollisionGeometry()){
      const auto name=link->getName();
      const auto padding=scene.getCollisionEnv()->getLinkPadding(name);
      const auto scale=scene.getCollisionEnv()->getLinkScale(name);
      if(padding!=0.) setLinkPadding(name,padding);
      if(scale!=1.) setLinkScale(name,scale);
    }
  }
  const auto& geometry() const{return robot_geoms_;}
};

float3 f3(const Vec& v){return make_float3(v.x(),v.y(),v.z());}
struct TransformRef {
  const moveit::core::LinkModel* link; int index;
};
struct ModelData {
  std::vector<Face> faces;
  std::vector<Part> parts;
  std::vector<TransformRef> transforms;
  struct Unsupported { TransformRef transform; Vec lo,hi; std::string name; };
  std::vector<Unsupported> fallback;
  std::vector<const moveit::core::LinkModel*> links;
  std::map<std::string,int> counts;
};
ModelData extract_model(const ModelGeometry& env,const Matrix& acm)
{
  ModelData data;
  for(const auto& entry:env.geometry()){
    const auto& shape=entry->collision_geometry_;
    const auto* ref=entry->collision_geometry_data_.get();
    if(ref->type!=collision_detection::BodyTypes::ROBOT_LINK) throw std::runtime_error("Unsupported attached geometry");
    const auto link=ref->ptr.link;
    auto found=std::find(data.links.begin(),data.links.end(),link);
    const int link_id=std::distance(data.links.begin(),found);
    if(found==data.links.end()) data.links.push_back(link);
    collision_detection::AllowedCollision::Type allowed{};
    if(acm.getAllowedCollision(link->getName(),"<octomap>",allowed)){
      if(allowed==collision_detection::AllowedCollision::ALWAYS) continue;
      if(allowed==collision_detection::AllowedCollision::CONDITIONAL) throw std::runtime_error("Conditional OctoMap ACM");
    }

    const int index=data.parts.size(), begin=data.faces.size();
    auto add=[&](const Vec& a,const Vec& b,const Vec& c){data.faces.push_back({f3(a),f3(b),f3(c),index});};
    bool solid=false;
    if(const auto* convex=dynamic_cast<const fcl::Convexd*>(shape.get())){
      const auto& v=convex->getVertices();
      const auto& f=convex->getFaces();
      for(size_t i=0;i<f.size();){
        const int n=f[i++];
        for(int j=1;j+1<n;++j) add(v[f[i]],v[f[i+j]],v[f[i+j+1]]);
        i+=n;
      }
      solid=true;
      ++data.counts["convex"];
    }else if(const auto* bvh=dynamic_cast<const fcl::BVHModel<fcl::OBBRSSd>*>(shape.get())){
      for(int i=0;i<bvh->num_tris;++i){
        const auto& t=bvh->tri_indices[i];
        add(bvh->vertices[t[0]],bvh->vertices[t[1]],bvh->vertices[t[2]]);
      }
      ++data.counts["triangle_mesh"];
    }else if(const auto* box=dynamic_cast<const fcl::Boxd*>(shape.get())){
      const Vec half=box->side/2;
      const auto at=[&](int i){return Vec(i&1?half.x():-half.x(), i&2?half.y():-half.y(), i&4?half.z():-half.z());};
      for(const auto& f:std::initializer_list<std::array<int,4>>{{0,2,3,1},{4,5,7,6},{0,4,6,2},{1,3,7,5},{0,1,5,4},{2,6,7,3}}){
        add(at(f[0]),at(f[1]),at(f[2])); add(at(f[0]),at(f[2]),at(f[3]));
      }
      solid=true;
      ++data.counts["box"];
    }else{
      data.fallback.push_back({{link,ref->shape_index},shape->aabb_local.min_,shape->aabb_local.max_,link->getName()});
      ++data.counts["cpu_primitive"];
      continue;
    }
    data.parts.push_back({f3(shape->aabb_local.min_),f3(shape->aabb_local.max_),begin,int(data.faces.size()),solid,link_id});
    data.transforms.push_back({link,ref->shape_index});
  }
  return data;
}
struct Occupancy {
  Grid grid;
  std::vector<int> cells;
  std::vector<float4> boxes;
  Vec low,high;
};
Occupancy occupied_boxes(const octomap::OcTree& octree)
{
  const auto* tree=&octree;
  Occupancy result;
  result.low=Vec::Constant(1e9);result.high=-result.low;
  for(auto it=tree->begin_leafs();it!=tree->end_leafs();++it){
    if(!tree->isNodeOccupied(*it)) continue;
    const Vec c(it.getX(),it.getY(),it.getZ());
    const double half=it.getSize()/2;
    result.low=result.low.cwiseMin(c-Vec::Constant(half));
    result.high=result.high.cwiseMax(c+Vec::Constant(half));
    result.boxes.push_back(make_float4(c.x(),c.y(),c.z(),half));
  }
  if(result.boxes.empty()) throw std::runtime_error("OctoMap has no occupied leaves");
  const double step=tree->getResolution();
  const auto dims=((result.high-result.low)/step).array().round().cast<int>().eval();
  result.grid={make_int3(dims[0],dims[1],dims[2]),f3(result.low),float(step)};
  return result;
}
void populate_grid(Occupancy& result)
{
  const auto& grid=result.grid;
  const std::array<int,3> dims={grid.size.x,grid.size.y,grid.size.z};
  const double step=grid.step;
  const auto cells=size_t(dims[0])*dims[1]*dims[2];
  if(cells==0 || cells>8*1024*1024) throw std::runtime_error("CUDA map exceeds dense grid budget");
  result.cells.assign(cells,-1);
  for(size_t i=0;i<result.boxes.size();++i){
    const auto& c=result.boxes[i];
    const Vec l=(Vec(c.x,c.y,c.z)-Vec::Constant(c.w)-result.low)/step;
    const int edge=std::lround(2*c.w/step);
    for(int z=std::lround(l.z());z<std::lround(l.z())+edge;++z)
      for(int y=std::lround(l.y());y<std::lround(l.y())+edge;++y)
        for(int x=std::lround(l.x());x<std::lround(l.x())+edge;++x) {
          auto& slot=result.cells.at((z*dims[1]+y)*dims[0]+x);
          if(slot!=-1) throw std::runtime_error("Overlapping occupancy cells");
          slot=i;
        }
  }
}
struct WorldTransforms {std::vector<Pose> gpu;std::vector<int> fallback;};
WorldTransforms prepare(const cleany::CadLinkage& linkage,const ModelData& geo,const Occupancy& map,
                        const Robot* input,size_t count,
                        const moveit::core::JointModelGroup* group)
{
  WorldTransforms result;
  result.gpu.resize(geo.transforms.size()*count);
  result.fallback.resize(count);
  for(size_t i=0;i<count;++i){
    Robot resolved(input[i]);
    if(!linkage.resolve(resolved)){
      result.fallback[i]=true;
      continue;
    }
    resolved.updateCollisionBodyTransforms();
    for(size_t g=0;g<geo.transforms.size();++g){
      const auto ref=geo.transforms[g];
      const auto& matrix=resolved.getCollisionBodyTransform(ref.link,ref.index).matrix();
      auto& pose=result.gpu[i*geo.transforms.size()+g];
      for(int row=0;row<3;++row)
        for(int col=0;col<4;++col) pose.r[row*4+col]=matrix(row,col);
    }
    for(const auto& part:geo.fallback){
      if(group && !group->getUpdatedLinkModelsSet().count(part.transform.link)) continue;
      const auto& t=resolved.getCollisionBodyTransform(part.transform.link,part.transform.index);
      const Vec center=t*((part.lo+part.hi)/2);
      const Vec reach=t.linear().cwiseAbs()*((part.hi-part.lo)/2);
      const auto& grid=map.grid;
      const auto lo=(((center-reach-map.low)/grid.step).array()-1.e-4).floor().cast<int>().eval();
      const auto hi=(((center+reach-map.low)/grid.step).array()+1.e-4).floor().cast<int>().eval();
      for(int z=std::max(0,lo[2]);z<=std::min(grid.size.z-1,hi[2]) && !result.fallback[i];++z)
        for(int y=std::max(0,lo[1]);y<=std::min(grid.size.y-1,hi[1]) && !result.fallback[i];++y)
          for(int x=std::max(0,lo[0]);x<=std::min(grid.size.x-1,hi[0]);++x){
            if(map.cells[(z*grid.size.y+y)*grid.size.x+x]>=0){result.fallback[i]=true;break;}
          }
    }
  }
  return result;
}

// The scene monitor's read lock covers the live occupancy tree as well as ACM.
struct GpuMapDeleter {
  void operator()(SceneGpu* ptr) const { if(ptr) free_gpu(ptr); }
};
bool same_map(const Occupancy& a,const Occupancy& b)
{
  return a.grid.step==b.grid.step && a.low==b.low && a.high==b.high &&
      a.boxes.size()==b.boxes.size() &&
      std::memcmp(a.boxes.data(),b.boxes.data(),a.boxes.size()*sizeof(float4))==0;
}
std::vector<double> geometry_signature(const Scene& scene)
{
  std::vector<double> values;
  for(const auto* link:scene.getRobotModel()->getLinkModelsWithCollisionGeometry()){
    const auto& name=link->getName();
    const auto& acm=scene.getAllowedCollisionMatrix();
    collision_detection::AllowedCollision::Type allow{};
    acm.getAllowedCollision(name,Scene::OCTOMAP_NS,allow);
    values.insert(values.end(),{scene.getCollisionEnv()->getLinkPadding(name),
        scene.getCollisionEnv()->getLinkScale(name),static_cast<double>(allow)});
  }
  return values;
}
std::vector<Robot> convert(const Scene& scene,const Service::Request& req)
{
  if(req.robot_states.empty() || req.robot_states.size()>64)
    throw std::runtime_error("Expected 1..64 full robot states");
  std::vector<Robot> states;
  states.reserve(req.robot_states.size());
  const auto& names=scene.getRobotModel()->getVariableNames();
  for(const auto& message:req.robot_states){
    if(!message.attached_collision_objects.empty() ||
       !message.multi_dof_joint_state.joint_names.empty() ||
       !message.multi_dof_joint_state.transforms.empty() ||
       message.joint_state.name.size()!=message.joint_state.position.size() ||
       std::any_of(message.joint_state.position.begin(),message.joint_state.position.end(),
                   [](double q){return !std::isfinite(q);}))
      throw std::runtime_error("Unsupported grasp state diff");
    std::set<std::string> unique;
    for(const auto& joint:message.joint_state.name)
      if(!unique.insert(joint).second || std::find(names.begin(),names.end(),joint)==names.end())
        throw std::runtime_error("Unknown or repeated joint in grasp state");
    Robot state=scene.getCurrentState();
    std::vector<const moveit::core::AttachedBody*> attached;
    state.getAttachedBodies(attached);
    if(!attached.empty() || !moveit::core::robotStateMsgToRobotState(message,state))
      throw std::runtime_error("Attached body or invalid grasp state");
    state.update();
    states.push_back(state);
  }
  return states;
}
}  // namespace

class GpuGraspOctomapCapability : public move_group::MoveGroupCapability
{
 public:
  GpuGraspOctomapCapability():MoveGroupCapability("GpuGraspOctomap") {}
  void initialize() override
  {
    const auto node=context_->moveit_cpp_->getNode();
    const auto create=[&](const char* name,bool compare) {
      return node->create_service<Service>(name,
        [this,node,compare](const std::shared_ptr<Service::Request> req,
                            const std::shared_ptr<Service::Response> out) {
          try {
            planning_scene_monitor::LockedPlanningSceneRO scene(context_->planning_scene_monitor_);
            check(*scene.operator->(),*req,*out,compare);
          } catch(const std::exception& e) {
            out->success=false;
            out->valid.clear();
            out->first_invalid_index=-1;
            out->message=e.what();
            RCLCPP_WARN_THROTTLE(node->get_logger(),*node->get_clock(),10000,
                                "CUDA grasp map fell back to MoveIt: %s",e.what());
          }
        });
    };
    service_=create("check_grasp_octomap",false);
    comparison_=create("compare_grasp_octomap",true);
  }

 private:
  void check(const Scene& scene,const Service::Request& req,Service::Response& out,bool compare)
  {
    const auto started=Clock::now();
    if(req.constraints!=moveit_msgs::msg::Constraints())
      throw std::runtime_error("Expected OctoMap query without constraints");
    const auto* group=req.group_name.empty()?nullptr:scene.getRobotModel()->getJointModelGroup(req.group_name);
    if(!req.group_name.empty() && !group)
      throw std::runtime_error("Unknown OctoMap query group");
    if(scene.getCollisionDetectorName()!=CalibratedCADCollisionAllocator::NAME)
      throw std::runtime_error("CUDA grasp collision requires the calibrated CAD model");
    const auto object=scene.getWorld()->getObject(Scene::OCTOMAP_NS);
    if(!object || object->shapes_.size()!=1 ||
       !(object->global_shape_poses_[0].matrix().isApprox(Eigen::Matrix4d::Identity(),1.e-12)))
      throw std::runtime_error("CUDA grasp collision requires one base-frame OctoMap");
    const auto* shape=dynamic_cast<const shapes::OcTree*>(object->shapes_.front().get());
    if(!shape || !shape->octree) throw std::runtime_error("Invalid OctoMap");
    auto occupancy=occupied_boxes(*shape->octree);
    auto signature=geometry_signature(scene);
    if(!linkage_ || scene.getRobotModel()!=robot_model_ || signature!=signature_){
      const char* calibration=std::getenv("CLEANY_CAD_LINKAGE_JSON");
      if(!calibration || !*calibration) throw std::runtime_error("CAD linkage unavailable");
      robot_model_=scene.getRobotModel();
      linkage_=std::make_unique<CadLinkage>(calibration,*robot_model_);
      model_data_=extract_model(ModelGeometry(scene),scene.getAllowedCollisionMatrix());
      signature_=std::move(signature);
      gpu_.reset();
    }
    if(!gpu_ || !same_map(occupancy_,occupancy)){
      populate_grid(occupancy);
      if(gpu_)
        update_gpu_map(gpu_.get(),occupancy.grid,occupancy.cells,occupancy.boxes);
      else
        gpu_.reset(load_gpu(model_data_.faces,model_data_.parts,occupancy.grid,
                            occupancy.cells,occupancy.boxes,64,model_data_.links.size()));
      occupancy_=std::move(occupancy);
      ++map_revision_;
    }
    auto states=convert(scene,req);
    const auto poses=prepare(*linkage_,model_data_,occupancy_,states.data(),states.size(),group);
    if(std::any_of(poses.fallback.begin(),poses.fallback.end(),[](int value){return value!=0;}))
      throw std::runtime_error("Unsupported collider or joint state in grasp batch");
    auto hits=query_gpu(gpu_.get(),poses.gpu.data(),states.size(),group!=nullptr);
    if(group){
      const auto& active=group->getUpdatedLinkModelsSet();
      const auto link_count=model_data_.links.size();
      std::vector<int> group_hits(states.size(),0);
      for(std::size_t link=0;link<link_count;++link){
        if(!active.count(model_data_.links[link])) continue;
        for(std::size_t row=0;row<states.size();++row)
          group_hits[row]|=hits[row*link_count+link];
      }
      hits=std::move(group_hits);
    }
    const auto gpu_seconds=std::chrono::duration<double>(Clock::now()-started).count();
    double moveit_seconds=0.;
    unsigned mismatches=0;
    if(compare){
      const auto began=Clock::now();
      auto map_scene=scene.diff();
      for(const auto& id:scene.getWorld()->getObjectIds())
        if(id!=Scene::OCTOMAP_NS) map_scene->getWorldNonConst()->removeObject(id);
      collision_detection::CollisionRequest request;
      request.group_name=req.group_name;
      for(std::size_t i=0;i<states.size();++i){
        collision_detection::CollisionResult result;
        map_scene->getCollisionEnv()->checkRobotCollision(
            request,result,states[i],map_scene->getAllowedCollisionMatrix());
        mismatches+=result.collision!=bool(hits[i]);
      }
      moveit_seconds=std::chrono::duration<double>(Clock::now()-began).count();
    }
    std::ostringstream metadata;
    metadata<<"{\"map_revision\":"<<map_revision_<<",\"gpu_seconds\":"<<gpu_seconds
            <<",\"moveit_seconds\":"<<moveit_seconds<<",\"mismatches\":"<<mismatches<<"}";
    out.message=metadata.str();
    if(mismatches) return;
    out.first_invalid_index=-1;
    for(std::size_t i=0;i<hits.size();++i){
      out.valid.push_back(!hits[i]);
      if(hits[i] && out.first_invalid_index<0) out.first_invalid_index=i;
    }
    out.success=true;
  }
  rclcpp::Service<Service>::SharedPtr service_,comparison_;
  moveit::core::RobotModelConstPtr robot_model_;
  std::unique_ptr<CadLinkage> linkage_;
  ModelData model_data_;
  std::vector<double> signature_;
  Occupancy occupancy_;
  std::unique_ptr<SceneGpu,GpuMapDeleter> gpu_;
  uint64_t map_revision_=0;
};
}  // namespace cleany

PLUGINLIB_EXPORT_CLASS(cleany::GpuGraspOctomapCapability,move_group::MoveGroupCapability)
