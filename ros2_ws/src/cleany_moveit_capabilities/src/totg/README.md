# MoveIt TOTG numerical correction

The private `cleany::totg` implementation is derived from MoveIt 2.5.9:

- [Source](https://github.com/moveit/moveit2/blob/2.5.9/moveit_core/trajectory_processing/src/time_optimal_trajectory_generation.cpp)
- [Header](https://github.com/moveit/moveit2/blob/2.5.9/moveit_core/trajectory_processing/include/moveit/trajectory_processing/time_optimal_trajectory_generation.h)

The original BSD license and copyright notices are retained in both files.
The namespace and include path isolate this implementation from the installed
MoveIt library. The numerical corrections are:

- Replace the fixed path-space epsilon with a per-trajectory epsilon bounded by
  switching-point separation and floating point resolution. This prevents finite
  differences from crossing narrow blends in the saved near-reversal regressions.
- Space output samples uniformly over the exact duration, avoiding a nearly
  duplicate final timestamp when duration rounds just above a sampling boundary.
- Bound integration steps by local curvature, deceleration and the next segment
  boundary in both directions. Boundary velocity uses the constant-acceleration
  squared-speed relation. Non-finite or stalled integration fails explicitly.
- Retain nonzero short segments and construct circular blends in local
  coordinates with `atan2`. Only numerically collinear turns are degenerate.
  Exact reversals and unblended corners have zero-speed boundary conditions;
  the same TOTG solver times each smooth section.
- Compute acceleration from the path tangent and curvature. Anchor output at
  blend midpoints and section endpoints, then subdivide each anchor interval.
  This retains short excursions without almost coincident event/grid samples.
- Refine quintic output until its Bernstein hull fits a capsule around an input
  segment and the model's position bounds. Half the path-tolerance budget is
  reserved for internal curve construction. Timestamp rounding is included in
  the hull; the wrapper bounds derivatives over the same duration uncertainty.

These changes do not widen path tolerance, change joint limits or provide a
second timing algorithm. Review upstream updates against these patches before
rebasing the private source.
