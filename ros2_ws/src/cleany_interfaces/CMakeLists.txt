cmake_minimum_required(VERSION 3.8)
project(cleany_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ComponentHealth.msg"
  "msg/SafetyStatus.msg"
  "msg/SystemFault.msg"
  "msg/SystemCapability.msg"
  "msg/MissionPermission.msg"
  "msg/SystemState.msg"
  "msg/SystemCapabilities.msg"
  "msg/SystemFaults.msg"
  "msg/SystemRecovery.msg"
  "msg/SystemRecoveries.msg"
  "msg/SystemHeartbeat.msg"
  "msg/SystemEvent.msg"
  "srv/CheckPermission.srv"
  "srv/RequestFaultReset.srv"
  "action/StopMission.action"
  "action/RecoverComponent.action"
  "action/RequestRecovery.action"
  "msg/TofFrame.msg"
  "action/DeskApproach.action"
  "msg/DetectedObject2D.msg"
  "msg/DetectedObject2DArray.msg"
  "msg/DetectedObject3D.msg"
  "msg/DetectedObject3DArray.msg"
  "msg/GraspCandidate.msg"
  "msg/ObservedObjectGeometry.msg"
  "msg/TrackedObjectReference.msg"

  "msg/RecordingCategory.msg"
  "msg/RecordingSource.msg"
  "msg/RecordingConfig.msg"
  "msg/RecordingCounter.msg"
  "msg/RecordingStatus.msg"
  "srv/RecordingControl.srv"
  "msg/MissionStatus.msg"
  "msg/DebugBTNode.msg"
  "msg/RuntimeDebugSnapshot.msg"
  "msg/RuntimeEvent.msg"

  "msg/MissionResult.msg"
  "msg/ManipulationExecutionRecord.msg"
  "action/ExecuteManipulationSkill.action"
  "action/InspectScene.action"
  "action/SelectReachableGrasp.action"
  "srv/PlanGrasp.srv"
  "srv/VerifyPlacement.srv"
  "srv/ObserveWristTarget.srv"
  "srv/OfferMission.srv"
  "srv/CancelMission.srv"
  "srv/GetRuntimeSnapshot.srv"
  "srv/GetManipulationExecution.srv"
  "srv/CancelManipulation.srv"
  "srv/GetSceneSnapshot.srv"
  "srv/RegisterPlacementTarget.srv"
  DEPENDENCIES geometry_msgs sensor_msgs shape_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # google-genai can install a newer anyio pytest plugin than the pytest
  # version bundled with ROS 2 Humble. Interface tests need no third-party
  # plugins, so keep discovery deterministic during configure and execution.
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  ament_add_pytest_test(
    interface_contract
    test/test_interface_contract.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
