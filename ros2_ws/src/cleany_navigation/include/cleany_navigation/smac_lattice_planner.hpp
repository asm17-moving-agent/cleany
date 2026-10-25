#pragma once
#include "nav2_smac_planner/smac_planner_lattice.hpp"

namespace cleany_navigation {
/// Smac Lattice with a collision-checked same-cell terminal connection.
/// Humble 1.1.20 cannot backtrace a search whose start is already the goal node.
class SmacLatticePlanner : public nav2_smac_planner::SmacPlannerLattice {
public:
#if CLEANY_NAV2_CANCEL_CHECKER
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::PoseStamped &, std::function<bool()>) override;
#else
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::PoseStamped &) override;
#endif
};
}
