#pragma once

#include <moveit/collision_detection_fcl/collision_env_fcl.h>
#include <moveit/collision_detection/collision_detector_allocator.h>
#include <fcl/geometry/shape/convex.h>
#include <geometric_shapes/shapes.h>

namespace cleany
{
/**
 * 닫힌 볼록 구성요소 메시를 요소 사이의 틈을 채우지 않고 입체로 분리합니다.
 * 좌표 단위는 미터입니다. 잘못되거나 열렸거나 오목한 구성요소는 예외를 던집니다.
 * 임의 CAD 표면에서 볼록 껍질을 추론하지 않으며 면 방향은 바깥쪽으로 정규화합니다.
 */
std::vector<std::unique_ptr<fcl::Convexd>> solid_components(const shapes::Mesh& mesh);

/**
 * 입체 로봇 메시로 FCL을 조회하며 world, ACM과 attachment 처리는 유지합니다.
 * 각 구성요소는 원래 link와 충돌 형상 변환 인덱스를 유지합니다.
 * 자기 충돌 조회는 구성요소를 순회하기 전에 겹치지 않거나 허용된 link 쌍을 제외합니다.
 * FCL 콜백, 거리 조회와 장면 복사는 기존 의미를 유지합니다.
 */
class SolidCollisionEnv : public collision_detection::CollisionEnvFCL
{
public:
  explicit SolidCollisionEnv(const moveit::core::RobotModelConstPtr& model,
                             double padding = 0., double scale = 1.);
  SolidCollisionEnv(const moveit::core::RobotModelConstPtr& model,
                    const collision_detection::WorldPtr& world, double padding = 0., double scale = 1.);
  SolidCollisionEnv(const SolidCollisionEnv& other, const collision_detection::WorldPtr& world);
  std::size_t solidComponentCount() const { return solid_components_; }
  void checkSelfCollision(const collision_detection::CollisionRequest& req,
                          collision_detection::CollisionResult& res,
                          const moveit::core::RobotState& state) const override;
  void checkSelfCollision(const collision_detection::CollisionRequest& req,
                          collision_detection::CollisionResult& res,
                          const moveit::core::RobotState& state,
                          const collision_detection::AllowedCollisionMatrix& acm) const override;
  using CollisionEnvFCL::checkRobotCollision;
  void checkRobotCollision(const collision_detection::CollisionRequest& req,
                           collision_detection::CollisionResult& res,
                           const moveit::core::RobotState& state) const override;
  void checkRobotCollision(const collision_detection::CollisionRequest& req,
                           collision_detection::CollisionResult& res,
                           const moveit::core::RobotState& state,
                           const collision_detection::AllowedCollisionMatrix& acm) const override;

protected:
  void updatedPaddingOrScaling(const std::vector<std::string>& links) override;

private:
  void rebuild();
  void checkGroupedSelf(const collision_detection::CollisionRequest& req,
                        collision_detection::CollisionResult& res,
                        const moveit::core::RobotState& state,
                        const collision_detection::AllowedCollisionMatrix* acm) const;
  std::size_t solid_components_ = 0;
};

class SolidCollisionAllocator
  : public collision_detection::CollisionDetectorAllocatorTemplate<SolidCollisionEnv, SolidCollisionAllocator>
{
public:
  static const std::string NAME;
};
}  // namespace cleany
