#pragma once

#include <chrono>
#include <stdexcept>
#include <thread>
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace cleany_navigation::mppi::test
{
/** @brief 짧은 단위 검사에서도 내부 executor 시작을 확인하는 실제 costmap입니다. */
class TestCostmap : public nav2_costmap_2d::Costmap2DROS
{
public:
  using nav2_costmap_2d::Costmap2DROS::Costmap2DROS;

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
  {
    const auto result = nav2_costmap_2d::Costmap2DROS::on_configure(state);
    if (result != nav2_util::CallbackReturn::SUCCESS) {
      return result;
    }
    // Humble NodeThread는 spin 시작 전에 cancel되면 종료 요청을 놓칠 수 있습니다.
    // 시간 지연 대신 실제 시작 상태를 확인해 즉시 파괴하는 검사도 안전하게 종료합니다.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!executor_->is_spinning()) {
      if (std::chrono::steady_clock::now() >= deadline) {
        throw std::runtime_error("Costmap executor did not start within 2 seconds");
      }
      std::this_thread::yield();
    }
    return result;
  }
};
}  // namespace cleany_navigation::mppi::test
