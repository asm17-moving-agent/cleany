cmake_minimum_required(VERSION 3.8)
project(cleany_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject2D.msg"
  "msg/DetectedObject2DArray.msg"
  "msg/DetectedObject3D.msg"
  "msg/DetectedObject3DArray.msg"
  "msg/GraspCandidate.msg"
  "msg/ObservedObjectGeometry.msg"
  "msg/TrackedObjectReference.msg"

  "msg/MissionStatus.msg"
  "msg/DebugBTNode.msg"
  "msg/RuntimeDebugSnapshot.msg"
  "msg/RuntimeEvent.msg"

  "msg/MissionResult.msg"
  "msg/ManipulationExecutionRecord.msg"
  "action/ExecuteManipulationSkill.action"
  "action/InspectScene.action"
  "action/SelectReachableGrasp.action"
  "action/ValidatePick.action"
  "srv/PlanGrasp.srv"
  "srv/ObserveObjectReference.srv"
  "srv/ObserveWristTarget.srv"
  "srv/OfferMission.srv"
  "srv/CancelMission.srv"
  "srv/GetRuntimeSnapshot.srv"
  "srv/GetManipulationExecution.srv"
  "srv/CancelManipulation.srv"
  "srv/GetSceneSnapshot.srv"
  "srv/RegisterPlacementTarget.srv"
  "srv/CheckManipulation.srv"
  "srv/VerifyPlacement.srv"
  "msg/WristTrackingStatus.msg"
  "msg/ManipulationOutcome.msg"
  "action/ReturnArm.action"
  "srv/ConfirmManipulationRecovery.srv"
  "srv/CheckStateValidityBatch.srv"
  DEPENDENCIES trajectory_msgs geometry_msgs sensor_msgs shape_msgs std_msgs moveit_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Interface tests need no third-party pytest plugins, so keep discovery
  # deterministic during configure and execution.
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  ament_add_pytest_test(
    interface_contract
    test/test_interface_contract.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
