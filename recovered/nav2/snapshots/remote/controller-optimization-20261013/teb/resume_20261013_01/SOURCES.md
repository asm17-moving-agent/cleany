# Source evidence

- [Nav2 1.1.20 ObstacleFootprintCritic](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_critics/src/obstacle_footprint.cpp): lethal, unknown, and off-grid rejection exceptions are distinct. Our diagnostic patch preserves those exceptions' text and logs the failed TEB pose/index.
- Local pinned TEB `src/optimal_planner.cpp`, `AddEdgesObstacles`: force inclusion and association cutoff are multiplied by min_obstacle_dist. A zero value excludes strictly positive polygon-obstacle distances. This is a static source finding; the raw obstacle exceptions and isolated positive-clearance pilot are separate measured evidence.
- Local pinned TEB `CMakeLists.txt` sets Release independently from the CMake cache field. `original_compile_flags.txt` and `dedicated_compile_flags.txt` both show -O3 -DNDEBUG.
- [Fortress Contact system](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/contact/Contact.cc): repeated collision elements are supported; the contact-specific topic is read inside the contact element. Messages publish only when contacts exist, at physics updates. update_rate is not enforced by this system. This informs the separately authorized relaxed-stage contact instrumentation.
- [Humble ros_gz Contacts conversion](https://github.com/gazebosim/ros_gz/blob/humble/ros_gz_bridge/src/convert/ros_gz_interfaces.cpp): physics collision entity names/IDs, positions, normals, depths and wrench fields are carried into ROS Contacts messages.

Results in REPORT.md come from preserved raw experiments, not from these documentation claims. Source compatibility patches are separate from controller tuning and from the relaxed-stage command delivery changes.
