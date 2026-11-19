#include <iostream>
#include <stdexcept>
#include "nav2_smac_planner/node_lattice.hpp"

// Parse with the installed Smac implementation without starting ROS or searching.
int main(int argc, char ** argv)
{
  if (argc != 2) {return 64;}
  nav2_smac_planner::SearchInfo info{};
  info.lattice_filepath = argv[1];
  info.allow_reverse_expansion = true;
  nav2_smac_planner::LatticeMotionTable table;
  unsigned width = 100;
  table.initMotionModel(width, info);
  size_t count = 0;
  for (const auto & group : table.motion_primitives) {count += group.size();}
  if (table.motion_primitives.size() != 16 || count != 224) {
    throw std::runtime_error("Unexpected primitive groups/count");
  }
  std::cout << "Native Smac parsed " << count << " primitives in "
            << table.motion_primitives.size() << " heading groups\n";
}
