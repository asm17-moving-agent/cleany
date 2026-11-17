#include <cleany_moveit_collision/cached_cad_collision_env.hpp>
#include <algorithm>
#include <array>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>

namespace cleany_cached
{
namespace
{
using Triangle = std::array<int, 3>;
struct Components
{
  explicit Components(std::size_t count) : parent(count) { std::iota(parent.begin(), parent.end(), 0); }
  int find(int i) { return parent[i] == i ? i : parent[i] = find(parent[i]); }
  void join(int a, int b) { parent[find(a)] = find(b); }
  std::vector<int> parent;
};

const moveit::core::LinkModel* rigidRoot(const moveit::core::LinkModel* link)
{
  while (link->getParentJointModel() && link->getParentLinkModel() &&
         link->getParentJointModel()->getType() == moveit::core::JointModel::FIXED)
    link = link->getParentLinkModel();
  return link;
}

bool sameRigidBody(const collision_detection::CollisionGeometryData* a,
                   const collision_detection::CollisionGeometryData* b)
{
  return a->type == collision_detection::BodyTypes::ROBOT_LINK &&
         b->type == collision_detection::BodyTypes::ROBOT_LINK &&
         rigidRoot(a->ptr.link) == rigidRoot(b->ptr.link);
}
}  // namespace

std::vector<std::unique_ptr<fcl::Convexd>> solid_components(const shapes::Mesh& mesh)
{
  if (mesh.vertex_count < 4 || mesh.triangle_count < 4)
    throw std::invalid_argument("Solid collision mesh is empty or degenerate");
  // STL readers may keep separate vertices for face normals. Welding identical
  // positions restores topology without moving any point or closing a gap.
  std::map<std::array<double, 3>, int> lookup;
  std::vector<Eigen::Vector3d> vertices;
  std::vector<int> indices(mesh.vertex_count);
  for (unsigned int i = 0; i < mesh.vertex_count; ++i)
  {
    std::array<double, 3> key{mesh.vertices[3*i], mesh.vertices[3*i+1], mesh.vertices[3*i+2]};
    if (!std::all_of(key.begin(), key.end(), [](double v) { return std::isfinite(v); }))
      throw std::invalid_argument("Nonfinite solid mesh vertex");
    auto found = lookup.emplace(key, static_cast<int>(vertices.size()));
    if (found.second) vertices.emplace_back(key[0], key[1], key[2]);
    indices[i] = found.first->second;
  }
  Components connected(vertices.size());
  std::vector<Triangle> triangles;
  for (unsigned int i = 0; i < mesh.triangle_count; ++i)
  {
    Triangle triangle;
    for (int j = 0; j < 3; ++j)
    {
      if (mesh.triangles[3*i+j] >= mesh.vertex_count) throw std::invalid_argument("Invalid mesh index");
      triangle[j] = indices[mesh.triangles[3*i+j]];
    }
    connected.join(triangle[0], triangle[1]);
    connected.join(triangle[0], triangle[2]);
    triangles.push_back(triangle);
  }
  std::map<int, std::vector<Triangle>> groups;
  for (const auto& triangle : triangles) groups[connected.find(triangle[0])].push_back(triangle);
  std::vector<std::unique_ptr<fcl::Convexd>> result;
  for (const auto& group : groups)
  {
    auto points = std::make_shared<std::vector<fcl::Vector3d>>();
    auto faces = std::make_shared<std::vector<int>>();
    std::map<int, int> remap;
    for (const auto& triangle : group.second)
      for (int i : triangle)
        if (remap.emplace(i, static_cast<int>(points->size())).second) points->push_back(vertices[i]);
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (const auto& point : *points) center += point;
    center /= points->size();
    std::map<std::pair<int, int>, int> edges;
    for (auto triangle : group.second)
    {
      for (int& i : triangle) i = remap.at(i);
      const auto& a = points->at(triangle[0]);
      Eigen::Vector3d normal = (points->at(triangle[1])-a).cross(points->at(triangle[2])-a);
      const double length = normal.norm();
      if (length <= 1e-18) throw std::invalid_argument("Degenerate solid mesh face");
      normal /= length;
      if (normal.dot(center-a) > 0.) { normal = -normal; std::swap(triangle[1], triangle[2]); }
      if (normal.dot(center-a) >= -1e-12) throw std::invalid_argument("Flat solid mesh component");
      for (const auto& point : *points)
        if (normal.dot(point-a) > 1e-9) throw std::invalid_argument("Concave mesh requires explicit convex decomposition");
      for (int i = 0; i < 3; ++i) ++edges[std::minmax(triangle[i], triangle[(i+1)%3])];
      faces->push_back(3);
      faces->insert(faces->end(), triangle.begin(), triangle.end());
    }
    if (static_cast<long>(points->size())-static_cast<long>(edges.size())+static_cast<long>(group.second.size()) != 2 ||
        !std::all_of(edges.begin(), edges.end(), [](const auto& edge) { return edge.second == 2; }))
      throw std::invalid_argument("Solid mesh must be a closed two-manifold");
    result.emplace_back(std::make_unique<fcl::Convexd>(points, group.second.size(), faces, true));
  }
  return result;
}

const std::string SolidCollisionAllocator::NAME = "cleany/SolidFCL";
const std::string CADInspectionCollisionAllocator::NAME = "cleany/CADInspectionFCL";

SolidCollisionEnv::SolidCollisionEnv(const moveit::core::RobotModelConstPtr& model, double padding, double scale,
    bool declared_only)
  : CollisionEnvFCL(model, padding, scale), declared_only_(declared_only) { rebuild(); }

SolidCollisionEnv::SolidCollisionEnv(const moveit::core::RobotModelConstPtr& model,
    const collision_detection::WorldPtr& world, double padding, double scale, bool declared_only)
  : CollisionEnvFCL(model, world, padding, scale), declared_only_(declared_only) { rebuild(); }

SolidCollisionEnv::SolidCollisionEnv(const SolidCollisionEnv& other, const collision_detection::WorldPtr& world)
  : CollisionEnvFCL(other, world), solid_components_(other.solid_components_), declared_only_(other.declared_only_),
    self_geoms_(other.self_geoms_), self_objects_(other.self_objects_),
    separate_self_geometry_(other.separate_self_geometry_) {}

void SolidCollisionEnv::updatedPaddingOrScaling(const std::vector<std::string>&) { rebuild(); }

void SolidCollisionEnv::checkSelfCollision(const collision_detection::CollisionRequest& req,
    collision_detection::CollisionResult& res, const moveit::core::RobotState& state) const
{
  checkGroupedSelf(req, res, state, nullptr);
}

void SolidCollisionEnv::checkSelfCollision(const collision_detection::CollisionRequest& req,
    collision_detection::CollisionResult& res, const moveit::core::RobotState& state,
    const collision_detection::AllowedCollisionMatrix& acm) const
{
  checkGroupedSelf(req, res, state, &acm);
}

void SolidCollisionEnv::checkRobotCollision(const collision_detection::CollisionRequest& req,
    collision_detection::CollisionResult& res, const moveit::core::RobotState& state) const
{
  if (!manager_->empty() || req.distance)
    CollisionEnvFCL::checkRobotCollision(req, res, state);
}

void SolidCollisionEnv::checkRobotCollision(const collision_detection::CollisionRequest& req,
    collision_detection::CollisionResult& res, const moveit::core::RobotState& state,
    const collision_detection::AllowedCollisionMatrix& acm) const
{
  if (!manager_->empty() || req.distance)
    CollisionEnvFCL::checkRobotCollision(req, res, state, acm);
}

void SolidCollisionEnv::checkGroupedSelf(const collision_detection::CollisionRequest& req,
    collision_detection::CollisionResult& res, const moveit::core::RobotState& state,
    const collision_detection::AllowedCollisionMatrix* acm) const
{
  using namespace collision_detection;
  // Distance requests retain FCL's distance traversal and result semantics.
  if (req.distance && !declared_only_)
  {
    CollisionEnvFCL::checkSelfCollisionHelper(req, res, state, acm);
    return;
  }
  FCLObject robot;
  constructSelfObjects(state, robot);
  struct Group
  {
    const CollisionGeometryData* owner;
    std::size_t begin, end;
    fcl::AABBd bounds;
    bool active;
  };
  CollisionData data(&req, &res, acm);
  data.enableGroup(getRobotModel());
  std::vector<Group> groups;
  groups.reserve(robot_model_->getLinkModelsWithCollisionGeometry().size());
  // Robot components and each attached body's components are contiguous. These
  // bounds are rebuilt from this query's state.
  for (std::size_t i = 0; i < robot.collision_objects_.size(); ++i)
  {
    const auto& object = robot.collision_objects_[i];
    const auto* owner = static_cast<const CollisionGeometryData*>(object->collisionGeometry()->getUserData());
    if (!groups.empty() && groups.back().owner->sameObject(*owner))
    {
      groups.back().end = i + 1;
      groups.back().bounds += object->getAABB();
    }
    else
    {
      const auto* link = owner->type == BodyTypes::ROBOT_LINK ? owner->ptr.link : owner->ptr.ab->getAttachedLink();
      const bool active = !data.active_components_only_ || data.active_components_only_->count(link);
      groups.push_back({owner, i, i + 1, object->getAABB(), active});
    }
  }
  for (std::size_t i = 0; i < groups.size() && !data.done_; ++i)
    for (std::size_t j = i + 1; j < groups.size() && !data.done_; ++j)
    {
      const auto& a = groups[i];
      const auto& b = groups[j];
      if ((!a.active && !b.active) || a.owner->sameObject(*b.owner) ||
          (declared_only_ && sameRigidBody(a.owner, b.owner)) || !a.bounds.overlap(b.bounds))
        continue;
      AllowedCollision::Type allowed;
      const bool has_entry = acm && acm->getAllowedCollision(a.owner->getID(), b.owner->getID(), allowed);
      if (has_entry && allowed == AllowedCollision::ALWAYS)
        continue;
      if (separate_self_geometry_ && !req.distance && (!has_entry || allowed == AllowedCollision::NEVER) &&
          a.owner->type == BodyTypes::ROBOT_LINK && b.owner->type == BodyTypes::ROBOT_LINK)
      {
        const auto* left = a.owner->ptr.link;
        const auto* right = b.owner->ptr.link;
        const auto* joint = left->getParentLinkModel() == right ? left->getParentJointModel() :
                            right->getParentLinkModel() == left ? right->getParentJointModel() : nullptr;
        if (joint)
        {
          const double* values = state.getJointPositions(joint);
          const std::vector<double> positions(values, values + joint->getVariableCount());
          {
            std::lock_guard<std::mutex> lock(clear_pair_mutex_);
            const auto found = clear_pairs_.find(joint);
            if (found != clear_pairs_.end() && found->second == positions) continue;
          }
          // Cache only a complete, unpermitted geometric clear result. Contact
          // limits, conditional ACM callbacks and world pose cannot weaken it.
          CollisionRequest probe_request;
          CollisionResult probe_result;
          CollisionData probe(&probe_request, &probe_result, nullptr);
          for (std::size_t x = a.begin; x < a.end && !probe.done_; ++x)
            for (std::size_t y = b.begin; y < b.end && !probe.done_; ++y)
            {
              auto* first = robot.collision_objects_[x].get();
              auto* second = robot.collision_objects_[y].get();
              if (first->getAABB().overlap(second->getAABB())) collisionCallback(first, second, &probe);
            }
          if (!probe_result.collision)
          {
            std::lock_guard<std::mutex> lock(clear_pair_mutex_);
            clear_pairs_[joint] = positions;
            continue;
          }
        }
      }
      for (std::size_t x = a.begin; x < a.end && !data.done_; ++x)
        for (std::size_t y = b.begin; y < b.end && !data.done_; ++y)
        {
          auto* first = robot.collision_objects_[x].get();
          auto* second = robot.collision_objects_[y].get();
          if (first->getAABB().overlap(second->getAABB()))
            collisionCallback(first, second, &data);
        }
    }
  if (req.distance)
  {
    DistanceRequest distance_request;
    distance_request.group_name = req.group_name;
    distance_request.acm = acm;
    distance_request.enableGroup(getRobotModel());
    DistanceResult distance_result;
    distanceSelf(distance_request, distance_result, state);
    res.distance = distance_result.minimum_distance.distance;
    if (req.detailed_distance) res.distance_result = distance_result;
  }
}

void SolidCollisionEnv::constructSelfObjects(const moveit::core::RobotState& state,
    collision_detection::FCLObject& objects) const
{
  // Preserve the upstream attachment ownership and lifetime handling.
  constructFCLObjectRobot(state, objects);
  if (!separate_self_geometry_) return;
  auto& items = objects.collision_objects_;
  items.erase(std::remove_if(items.begin(), items.end(), [](const auto& object) {
    const auto* owner = static_cast<const collision_detection::CollisionGeometryData*>(
        object->collisionGeometry()->getUserData());
    return owner->type == collision_detection::BodyTypes::ROBOT_LINK;
  }), items.end());
  for (std::size_t index = 0; index < self_geoms_.size(); ++index)
  {
    const auto& geometry = self_geoms_[index];
    const auto& owner = geometry->collision_geometry_data_;
    // The geometry constructor recomputes a mesh's local AABB. Copy its cached
    // object as upstream FCL does, and update only the state-dependent bounds.
    auto object = std::make_shared<fcl::CollisionObjectd>(*self_objects_[index]);
    object->setTransform(state.getCollisionBodyTransform(owner->ptr.link, owner->shape_index));
    object->computeAABB();
    items.push_back(std::move(object));
  }
}

void SolidCollisionEnv::distanceSelf(const collision_detection::DistanceRequest& req,
    collision_detection::DistanceResult& res, const moveit::core::RobotState& state) const
{
  if (!declared_only_)
  {
    CollisionEnvFCL::distanceSelf(req, res, state);
    return;
  }
  collision_detection::FCLObject objects;
  constructSelfObjects(state, objects);
  fcl::DynamicAABBTreeCollisionManagerd manager;
  objects.registerTo(&manager);
  manager.setup();
  collision_detection::DistanceData data(&req, &res);
  manager.distance(&data, [](fcl::CollisionObjectd* a, fcl::CollisionObjectd* b,
                            void* context, double& distance) {
    using collision_detection::CollisionGeometryData;
    if (sameRigidBody(static_cast<const CollisionGeometryData*>(a->collisionGeometry()->getUserData()),
                      static_cast<const CollisionGeometryData*>(b->collisionGeometry()->getUserData())))
      return false;
    return collision_detection::distanceCallback(a, b, context, distance);
  });
}

void SolidCollisionEnv::rebuild()
{
  std::vector<collision_detection::FCLGeometryConstPtr> geometries;
  std::vector<collision_detection::FCLCollisionObjectConstPtr> objects;
  std::vector<collision_detection::FCLGeometryConstPtr> self_geometries;
  std::vector<collision_detection::FCLCollisionObjectConstPtr> self_objects;
  bool separate_self = false;
  std::set<std::string> self_links, world_links;
  std::size_t count = 0;
  for (const auto* link : robot_model_->getLinkModelsWithCollisionGeometry())
    for (std::size_t index = 0; index < link->getShapes().size(); ++index)
    {
      const auto& shape = link->getShapes()[index];
      const double scale = getLinkScale(link->getName()), padding = getLinkPadding(link->getName());
      bool use_solid = !declared_only_;
      bool self_only = false, world_only = false;
      if (declared_only_)
      {
        const auto urdf_link = robot_model_->getURDF()->getLink(link->getName());
        if (!urdf_link || urdf_link->collision_array.size() != link->getShapes().size())
          throw std::runtime_error("CAD collision declarations do not match loaded shapes");
        auto name = urdf_link->collision_array[index]->name;
        self_only = name.rfind("cad_self__", 0) == 0;
        world_only = name.rfind("cad_world__", 0) == 0;
        if (self_only) name.erase(0, std::string("cad_self__").size());
        if (world_only) name.erase(0, std::string("cad_world__").size());
        separate_self |= self_only || world_only;
        if (self_only) self_links.insert(link->getName());
        if (world_only) world_links.insert(link->getName());
        use_solid = name.rfind("cad_enclosure__", 0) == 0;
        if (use_solid && shape->type != shapes::MESH)
          throw std::runtime_error("A declared CAD enclosure must be a convex mesh");
      }
      auto add_geometry = [&](const collision_detection::FCLGeometryConstPtr& geometry) {
        if (!geometry) throw std::runtime_error("Unable to construct CAD collision geometry");
        if (!self_only) geometries.push_back(geometry);
        if (!world_only) self_geometries.push_back(geometry);
      };
      if (shape->type == shapes::MESH && use_solid)
      {
        std::unique_ptr<shapes::Mesh> mesh(static_cast<const shapes::Mesh*>(shape.get())->clone());
        mesh->scaleAndPadd(scale, padding);
        try
        {
          auto components = solid_components(*mesh);
          count += components.size();
          for (auto& component : components)
            add_geometry(std::make_shared<collision_detection::FCLGeometry>(component.release(), link, index));
        }
        catch (const std::exception& error)
        {
          throw std::runtime_error("Solid collision geometry rejected for " + link->getName() +
                                   " shape " + std::to_string(index) + ": " + error.what());
        }
      }
      else
        add_geometry(collision_detection::createCollisionGeometry(shape, scale, padding, link, index));
    }
  if (self_links != world_links)
    throw std::runtime_error("Each CAD self/world override requires both representations on the same link");
  for (const auto& geometry : geometries)
  {
    if (!geometry) throw std::runtime_error("Unable to construct solid collision geometry");
    objects.push_back(std::make_shared<const fcl::CollisionObjectd>(geometry->collision_geometry_));
  }
  for (const auto& geometry : self_geometries)
    self_objects.push_back(std::make_shared<const fcl::CollisionObjectd>(geometry->collision_geometry_));
  // FCL constructs each state object from this metadata, not the vector index.
  // Preparing both vectors first leaves the old geometry intact on rejection.
  robot_geoms_.swap(geometries);
  robot_fcl_objs_.swap(objects);
  self_geoms_.swap(self_geometries);
  self_objects_.swap(self_objects);
  separate_self_geometry_ = separate_self;
  solid_components_ = count;
  std::lock_guard<std::mutex> lock(clear_pair_mutex_);
  clear_pairs_.clear();
}
}  // namespace cleany_cached
