#include <cleany_moveit_capabilities/solid_collision_env.hpp>
#include <gtest/gtest.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <octomap/OcTree.h>
#include <cstring>
#include <cmath>

namespace
{
std::shared_ptr<shapes::Mesh> two_boxes()
{
  std::unique_ptr<shapes::Mesh> box(shapes::createMeshFromShape(shapes::Box(.06,.06,.06)));
  auto mesh = std::make_shared<shapes::Mesh>(box->vertex_count*2,box->triangle_count*2);
  for (unsigned int part=0;part<2;++part)
  {
    for (unsigned int i=0;i<box->vertex_count;++i)
      for (unsigned int axis=0;axis<3;++axis)
        mesh->vertices[3*(part*box->vertex_count+i)+axis]=box->vertices[3*i+axis]+(axis==0 ? (part ? .08 : -.08) : 0.);
    for (unsigned int i=0;i<box->triangle_count*3;++i)
      mesh->triangles[part*box->triangle_count*3+i]=box->triangles[i]+part*box->vertex_count;
  }
  mesh->computeTriangleNormals(); mesh->computeVertexNormals();
  return mesh;
}

class SolidCollisionTest : public testing::Test
{
protected:
  void SetUp() override
  {
    auto urdf=urdf::parseURDF(R"(<robot name="solid_test"><link name="base"/><link name="tool">
      <collision><origin xyz="0 0.2 0" rpy="0 0 0.4"/><geometry><box size=".1 .1 .1"/></geometry></collision></link>
      <joint name="slide" type="prismatic"><parent link="base"/><child link="tool"/>
      <axis xyz="1 0 0"/><limit lower="-2" upper="2" effort="1" velocity="1"/></joint></robot>)");
    auto srdf=std::make_shared<srdf::Model>();
    ASSERT_TRUE(srdf->initString(*urdf,"<robot name='solid_test'><group name='arm'><joint name='slide'/></group></robot>"));
    robot=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
    auto* link=robot->getLinkModel("tool");
    auto origins=link->getCollisionOriginTransforms();
    link->setGeometry({two_boxes()},origins);
    scene=std::make_shared<planning_scene::PlanningScene>(robot);
    scene->getCurrentStateNonConst().setToDefaultValues();
    scene->getCurrentStateNonConst().update();
  }
  Eigen::Vector3d point(double x) const
  {
    return scene->getCurrentState().getCollisionBodyTransform(robot->getLinkModel("tool"),0)*Eigen::Vector3d(x,0,0);
  }
  void sphere(double x)
  {
    scene->getWorldNonConst()->clearObjects();
    Eigen::Isometry3d pose=Eigen::Isometry3d::Identity(); pose.translation()=point(x);
    scene->getWorldNonConst()->addToObject("probe",std::make_shared<shapes::Sphere>(.003),pose);
  }
  collision_detection::CollisionResult check() const
  {
    collision_detection::CollisionRequest req; req.contacts=true; req.max_contacts=100;
    collision_detection::CollisionResult res; scene->checkCollision(req,res);
    return res;
  }
  void solid() { scene->allocateCollisionDetector(cleany::SolidCollisionAllocator::create()); }
  moveit::core::RobotModelPtr robot;
  planning_scene::PlanningScenePtr scene;
};

TEST_F(SolidCollisionTest, DetectsContainedSphereAndPreservesDisconnectedGap)
{
  sphere(-.08);
  EXPECT_FALSE(check().collision);  // The original triangle-surface detector misses this.
  solid();
  ASSERT_TRUE(check().collision);
  EXPECT_EQ(check().contacts.begin()->first,std::make_pair(std::string("probe"),std::string("tool")));
  const auto* env=dynamic_cast<const cleany::SolidCollisionEnv*>(scene->getCollisionEnv().get());
  ASSERT_NE(env,nullptr); EXPECT_EQ(env->solidComponentCount(),2u);
  sphere(0.); EXPECT_FALSE(check().collision);
  sphere(.08); EXPECT_TRUE(check().collision);
  sphere(.15); EXPECT_FALSE(check().collision);
}

TEST_F(SolidCollisionTest, LiveAcmGroupsTransformsAndSceneCopiesAreRespected)
{
  solid(); sphere(-.08);
  scene->getAllowedCollisionMatrixNonConst().setEntry("probe","tool",true);
  EXPECT_FALSE(check().collision);
  scene->getAllowedCollisionMatrixNonConst().setEntry("probe","tool",false);
  collision_detection::CollisionRequest req; req.group_name="arm";
  collision_detection::CollisionResult result;
  scene->checkCollision(req,result); EXPECT_TRUE(result.collision);
  auto copy=planning_scene::PlanningScene::clone(scene);
  result.clear(); copy->checkCollision(req,result); EXPECT_TRUE(result.collision);
  scene->getCurrentStateNonConst().setVariablePosition("slide",.5);
  scene->getCurrentStateNonConst().update(); EXPECT_FALSE(check().collision);
}

TEST_F(SolidCollisionTest, PaddingScalingDistanceAndWorldUpdatesUseSolidGeometry)
{
  solid(); sphere(.12);
  EXPECT_FALSE(check().collision);
  EXPECT_GT(scene->distanceToCollision(scene->getCurrentState()),0.);
  scene->getCollisionEnvNonConst()->setLinkScale("tool",1.2);
  EXPECT_TRUE(check().collision);
  scene->getCollisionEnvNonConst()->setLinkScale("tool",1.);
  EXPECT_FALSE(check().collision);
  scene->getCollisionEnvNonConst()->setLinkPadding("tool",.02);
  EXPECT_TRUE(check().collision);
  scene->getCollisionEnvNonConst()->setLinkPadding("tool",0.);
  EXPECT_FALSE(check().collision);
  sphere(-.08); EXPECT_LE(scene->distanceToCollision(scene->getCurrentState()),0.);
  scene->getWorldNonConst()->removeObject("probe"); EXPECT_FALSE(check().collision);
}

TEST_F(SolidCollisionTest, OccupiedVoxelAndAttachedPayloadInsideRobotAreRejected)
{
  solid();
  auto tree=std::make_shared<octomap::OcTree>(.001);
  const auto p=point(-.08);
  tree->updateNode(octomap::point3d(p.x(),p.y(),p.z()),true);
  scene->processOctomapPtr(tree,Eigen::Isometry3d::Identity());
  EXPECT_TRUE(check().collision);
  scene->getAllowedCollisionMatrixNonConst().setEntry("tool","<octomap>",true);
  EXPECT_FALSE(check().collision);
  scene->getWorldNonConst()->clearObjects();
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name="base"; attached.touch_links={"base"};
  attached.object.id="payload"; attached.object.header.frame_id="base";
  shape_msgs::msg::SolidPrimitive shape; shape.type=shape_msgs::msg::SolidPrimitive::SPHERE; shape.dimensions={.003};
  attached.object.primitives.push_back(shape);
  geometry_msgs::msg::Pose pose; pose.orientation.w=1.; pose.position.x=p.x(); pose.position.y=p.y(); pose.position.z=p.z();
  attached.object.primitive_poses.push_back(pose);
  ASSERT_TRUE(scene->processAttachedCollisionObjectMsg(attached));
  EXPECT_TRUE(check().collision);
  scene->getAllowedCollisionMatrixNonConst().setEntry("tool","payload",true);
  EXPECT_FALSE(check().collision);
}

TEST(SolidComponents, InvalidMeshDoesNotSilentlyBecomeAConvexHull)
{
  auto mesh=two_boxes();
  mesh->vertices[0]=0.; mesh->vertices[1]=0.; mesh->vertices[2]=0.;
  EXPECT_THROW(cleany::solid_components(*mesh),std::invalid_argument);
  mesh=two_boxes(); --mesh->triangle_count;
  EXPECT_THROW(cleany::solid_components(*mesh),std::invalid_argument);
}
}  // namespace

TEST(SolidBroadphase, MatchesFclForChangingStatesGroupsAndConditionalAcm)
{
  auto urdf=urdf::parseURDF(R"(<robot name="group_test"><link name="base"/>
    <link name="a"><collision><geometry><box size=".1 .1 .1"/></geometry></collision></link>
    <link name="b"><collision><geometry><box size=".1 .1 .1"/></geometry></collision></link>
    <link name="c"><collision><geometry><box size=".01 .01 .01"/></geometry></collision></link>
    <joint name="x" type="prismatic"><parent link="base"/><child link="a"/>
      <axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
    <joint name="y" type="prismatic"><parent link="base"/><child link="b"/>
      <origin rpy="0.2 0.3 0.4"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
    <joint name="z" type="prismatic"><parent link="base"/><child link="c"/>
      <origin xyz="0.08 0 0"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
    </robot>)");
  auto srdf=std::make_shared<srdf::Model>();
  ASSERT_TRUE(srdf->initString(*urdf,"<robot name='group_test'><group name='only_a'><joint name='x'/></group></robot>"));
  auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
  for(const std::string name : {"a","b"})
  {
    auto* link=model->getLinkModel(name);
    link->setGeometry({two_boxes()},link->getCollisionOriginTransforms());
  }
  cleany::SolidCollisionEnv env(model);
  moveit::core::RobotState state(model);
  unsigned int positives=0,negatives=0;
  for(int i=0;i<160;++i)
  {
    state.setToDefaultValues();
    state.setVariablePosition("x",.12*std::sin(i*.31));
    state.setVariablePosition("y",.12*std::cos(i*.53));
    state.setVariablePosition("z",.08*std::sin(i*.71));
    state.update();
    collision_detection::AllowedCollisionMatrix acm;
    if(i%4==1) acm.setEntry("a","b",true);
    if(i%4==2 || i%4==3)
    {
      collision_detection::DecideContactFn decide=
        [i](const collision_detection::Contact& contact){return i%4==2 && contact.depth<.02;};
      acm.setEntry("a","b",decide);
    }
    for(bool contacts : {false,true})
      for(const std::string group : {"","only_a"})
      {
        collision_detection::CollisionRequest req;
        req.contacts=contacts; req.max_contacts=1000; req.max_contacts_per_pair=100;
        req.group_name=group; req.cost=i%7==0; req.max_cost_sources=1000;
        req.distance=i%11==0; req.detailed_distance=req.distance;
        collision_detection::CollisionResult expected,actual;
        env.CollisionEnvFCL::checkSelfCollision(req,expected,state,acm);
        env.checkSelfCollision(req,actual,state,acm);
        ASSERT_EQ(actual.collision,expected.collision) << i << " " << group;
        positives+=actual.collision;negatives+=!actual.collision;
        if(contacts)
        {
          EXPECT_EQ(actual.contacts.size(),expected.contacts.size());
          for(const auto& pair : expected.contacts) EXPECT_TRUE(actual.contacts.count(pair.first));
        }
        if(req.distance) EXPECT_DOUBLE_EQ(actual.distance,expected.distance);
        EXPECT_EQ(actual.cost_sources.size(),expected.cost_sources.size());
        if(i%5==0)
        {
          expected.clear();actual.clear();
          env.CollisionEnvFCL::checkSelfCollision(req,expected,state);
          env.checkSelfCollision(req,actual,state);
          EXPECT_EQ(actual.collision,expected.collision);
        }
      }
  }
  EXPECT_GT(positives,0u);EXPECT_GT(negatives,0u);
}
