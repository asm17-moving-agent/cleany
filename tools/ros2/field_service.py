"""Start Jetson field components. Check mode validates configuration without opening devices."""

import argparse
import hashlib
import json
import math
import os
import sqlite3
import subprocess
import sys
import tempfile
import time
from datetime import datetime
from pathlib import Path

import yaml

ROLES = ("rgbd", "perception", "navigation", "collection", "mission", "pose")
CONFIG = Path(os.environ.get("CLEANY_FIELD_CONFIG_DIR", "/etc/opt/cleany/field"))
STATE = Path(os.environ.get("CLEANY_FIELD_STATE_DIR", "/var/opt/cleany/state/cleany"))
RUN = Path(os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")) / "cleany-field"
REPO = Path(os.environ.get("CLEANY_FIELD_REPO", "/opt/cleany/current/field/repo"))
OVERLAY = Path(os.environ.get("CLEANY_FIELD_INSTALL", "/opt/cleany/current/field/install"))
MAP = Path(os.environ.get("CLEANY_FIELD_MAP", "/var/opt/cleany/robot/map/source.yaml"))
TARGETS = CONFIG / "field-targets.json"
COLLECTION = CONFIG / "field-collection.json"
TOKEN = Path(os.environ.get("CLEANY_DEVICE_TOKEN_FILE", "/etc/opt/cleany/control-plane/cleany-01.token"))
DDS = Path(os.environ.get("CLEANY_FIELD_DDS_FILE", "/etc/opt/cleany/field/fastdds.xml"))
FIELD_RMW = os.environ.get("CLEANY_FIELD_RMW", "rmw_fastrtps_cpp")
JOINTS = ("shoulder_yaw", "shoulder_pitch", "elbow_pitch", "wrist_pitch", "wrist_roll")


def park_pose(config):
    """Use measured stow, bin and empty-open gripper positions from the field profile."""
    if (config.get("commissioning_verified") is not True or config.get("operation_mode") != "full_collection"
            or "demo_trash_bin" not in config["label_destinations"].values()):
        raise ValueError("field collection configuration")
    park = {}
    for side in ("left", "right"):
        arm = config["arms"][side]
        for angles in (arm["home_joint_rad"], arm["bins"]["demo_trash_bin"]):
            if (not isinstance(angles, list) or len(angles) != len(JOINTS)
                    or not all(type(q) in (int, float) and math.isfinite(q) for q in angles)):
                raise ValueError("field arm pose")
        opened = arm["gripper"]["open_joint_rad"]
        if type(opened) not in (int, float) or not math.isfinite(opened):
            raise ValueError("empty gripper pose")
        for name, angle in zip((*JOINTS, "gripper"), [*arm["home_joint_rad"], opened]):
            park[f"{side}_{name}_joint"] = angle
    return park


def bound_targets():
    from cleany_mission_manager.adapters.config import load_targets

    targets = load_targets(str(TARGETS))
    metadata = json.loads((MAP.parent / "metadata.json").read_text())
    if (not targets.binding or not targets.seats or targets.map_id != metadata["map_id"]
            or targets.frame_id != metadata["frame_id"]):
        raise ValueError("field targets")
    pose_map_id = os.environ.get("CLEANY_MAP_ID")
    if pose_map_id and pose_map_id != metadata["map_id"]:
        raise ValueError("pose observer map ID")
    hashes = metadata["source_sha256"]
    image = MAP.parent / yaml.safe_load(MAP.read_text())["image"]
    pgm_hash = hashlib.sha256(image.read_bytes()).hexdigest()
    if pgm_hash != hashes["map.pgm"] or pgm_hash != targets.binding["source_sha256"]:
        raise ValueError("map digest")
    if hashlib.sha256(MAP.read_bytes()).hexdigest() != hashes.get(MAP.name, hashes.get("map.yaml")):
        raise ValueError("map origin or resolution changed")
    if "nav2" in metadata and metadata["nav2"]["grid_revision"] != os.environ.get("CLEANY_MISSION_GRID_REVISION"):
        raise ValueError("field grid revision")
    return targets


def head_camera_serial():
    profile = REPO / "ros2_ws/src/cleany_perception/config/head_rgb_intrinsics.yaml"
    serial = yaml.safe_load(profile.read_text())["serial_no"]
    if not isinstance(serial, str) or not serial.isdigit():
        raise ValueError("missing calibrated head camera serial")
    return serial


def blockers(role):
    errors = []
    if role == "rgbd":
        files = (CONFIG / "head-rgb-intrinsics.json", REPO / "ros2_ws/src/cleany_perception/config/realsense_pregrasp.yaml")
        try:
            head_camera_serial()
            if not all(path.is_file() for path in files):
                raise ValueError("missing RGB-D calibration")
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
            errors.append("RGB-D 카메라 일련번호, 보정 파일과 프로필 확인 필요")
    if role == "perception":
        try:
            subprocess.run([REPO / "tools/perception-native", "--check"], check=True,
                           env={**os.environ, "CLEANY_INSTALL_BASE": str(OVERLAY)})
        except (OSError, subprocess.CalledProcessError):
            errors.append("인식 모델과 native 환경 확인 필요")
    if role in ("navigation", "mission", "pose"):
        try:
            bound_targets()
        except (OSError, ValueError, TypeError, KeyError, AttributeError, yaml.YAMLError):
            errors.append("현장 승인 좌석·Home과 운영 SLAM 지도 일치 확인 필요")
    if role in ("collection", "mission"):
        try:
            park_pose(json.loads(COLLECTION.read_text()))
        except (OSError, ValueError, KeyError, TypeError, AttributeError):
            errors.append("양팔 수납·수거함 투입·빈 그리퍼 실측 설정 필요")
    if role in ("navigation", "collection"):
        topics = [os.environ.get(key, "") for key in
                  ("CLEANY_ESTOP_RELEASED_TOPIC", "CLEANY_PROTECTION_READY_TOPIC")]
        if any(not t.startswith("/") for t in topics) or topics[0] == topics[1]:
            errors.append("독립 비상정지·보호 입력 토픽 설정 필요")
    if role == "navigation":
        try:
            profile = yaml.safe_load(Path(os.environ["CLEANY_BASE_PROFILE"]).read_text())
            for component in ("lidar", "agent"):
                if not Path(profile[component]["serial_port"]).is_char_device():
                    errors.append(f"{component} 시리얼 장치 대기")
        except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError):
            errors.append("실물 베이스 프로필 확인 필요")
        if not os.environ.get("CLEANY_MISSION_GRID_REVISION"):
            errors.append("점유 격자 revision 설정 필요")
    if role == "mission" and (not TOKEN.is_file() or not TOKEN.stat().st_size):
        errors.append("Gateway 기기 등록 필요")
    return errors


def write_status(role, reasons):
    RUN.mkdir(mode=0o700, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode="w", dir=RUN, delete=False) as out:
        json.dump({"blockers": reasons, "updated": time.time()}, out, ensure_ascii=False)
    os.replace(out.name, RUN / f"{role}.json")


def check_execution_journal():
    """Inspect the arm owner's durable history before enabling torque after a restart."""
    path = STATE / "manipulation_jetson/executions.sqlite3"
    if not path.exists():
        return
    from cleany_manipulation_bt.store import record_digest
    from cleany_skill_executor.manipulation.models import Record, RecordState
    from cleany_skill_executor.manipulation.store import inhibits_execution

    with sqlite3.connect(path.as_uri() + "?mode=ro", uri=True) as db:
        for (payload,) in db.execute("SELECT payload FROM executions"):
            record = Record.from_dict(json.loads(payload))
            if record.record_state == RecordState.ACTIVE:
                raise ValueError("팔 실행 도중 중단됨, 현장 팔 복구 필요")
            if inhibits_execution(record) and not db.execute(
                "SELECT 1 FROM recovery_confirmations WHERE execution_id=? AND revision=? AND payload_sha256=?",
                (record.goal.execution_id, record.revision, record_digest(record)),
            ).fetchone():
                raise ValueError("팔 실행의 물체·정지 상태 확인 및 현장 팔 복구 필요")


def dashboard_database():
    return Path(os.environ.get("CLEANY_DATABASE_PATH", "/var/opt/cleany/control-plane/control-plane.db"))


def check_arm_journals():
    """Hold physical power-up until the interrupted seat was inspected on site."""
    check_execution_journal()
    journal = STATE / "desk-field-missions.db"
    backend = dashboard_database()
    with sqlite3.connect(backend.as_uri() + "?mode=ro", uri=True) as dashboard:
        has_confirmations = dashboard.execute(
            "SELECT 1 FROM sqlite_master WHERE type='table' AND name='field_recovery_confirmations'"
        ).fetchone()
        confirmed = ({row[0] for row in dashboard.execute(
            "SELECT mission_id FROM field_recovery_confirmations"
        )} if has_confirmations else set())
        pending = dashboard.execute("""
            SELECT mission_id, active FROM missions WHERE robot_id = ?
            AND (active=1 OR json_extract(payload,'$.outcome')='INTERRUPTED')""",
            (os.environ.get("CLEANY_ROBOT_ID", "cleany-01"),))
        if any(active or mission_id not in confirmed for mission_id, active in pending):
            raise ValueError("중단 좌석과 로봇 상태를 현장에서 확인하세요")
        if journal.exists():
            with sqlite3.connect(journal.as_uri() + "?mode=ro", uri=True) as db:
                fault = db.execute("SELECT value FROM metadata WHERE key='error_code'").fetchone()
                if fault and fault[0] in ("E_STOP", "HARDWARE_ERROR"):
                    raise ValueError("비상정지 또는 하드웨어 고장의 현장 복구 필요")
                for mid, report in db.execute("SELECT mission_id,report FROM missions ORDER BY rowid DESC LIMIT 1"):
                    if report is None or (json.loads(report)["outcome"] == "INTERRUPTED" and mid not in confirmed):
                        raise ValueError("중단 미션의 현장 확인과 Gateway 동기화 대기")


def safe_inputs():
    """Require a fresh assertion from both field protection inputs after subscribing."""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Bool

    rclpy.init()
    node = rclpy.create_node("field_arm_boot_check")
    samples = {}
    topics = [os.environ[k] for k in ("CLEANY_ESTOP_RELEASED_TOPIC", "CLEANY_PROTECTION_READY_TOPIC")]
    def update(topic, msg):
        samples[topic] = (msg.data, time.monotonic())
    try:
        for topic in topics:
            node.create_subscription(Bool, topic, lambda msg, t=topic: update(t, msg), qos_profile_sensor_data)
        until = time.monotonic() + 4
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(t in samples and samples[t][0] and time.monotonic() - samples[t][1] < 1
                   and node.count_publishers(t) == 1 for t in topics):
                return
        raise ValueError("독립 비상정지·보호 입력의 최신 해제 신호 대기")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def collection():
    from ament_index_python.packages import get_package_prefix

    driver = Path(get_package_prefix("cleany_robot_interface")) / "lib/libcleany_feetech_hardware.so"
    if b"CLEANY_FIELD_PARK_JOINTS" not in driver.read_bytes():
        raise ValueError("부팅 자세 검사를 포함한 팔 드라이버 설치 필요")
    check_arm_journals()
    safe_inputs()
    pose = park_pose(json.loads(COLLECTION.read_text()))
    os.environ.update(CLEANY_FIELD_PARK_JOINTS=" ".join(f"{k} {v}" for k, v in pose.items()),
                      CLEANY_FIELD_ARM_CHECK=str(Path(__file__).with_name("field-service")),
                      CLEANY_MANIPULATION_COLLECTION_CONFIG=str(COLLECTION),
                      CLEANY_COLLECTION_USE_EXISTING_PERCEPTION="true",
                      CLEANY_COLLECTION_SHARED_RMW_IMPLEMENTATION=FIELD_RMW,
                      CLEANY_INSTALL_BASE=str(OVERLAY),
                      CLEANY_COLLECTION_STATE_DIR=str(RUN / "collection-processes"))
    for key in ("CLEANY_ROBOT_STATE_PROFILE", "CLEANY_FEETECH_CONTROL_CONFIG", "CLEANY_HARDWARE_MONITOR_CONFIG"):
        if not os.environ.get(key) or not Path(os.environ[key]).is_file():
            raise ValueError("운영 팔 모델·제어·모니터 설정 확인 필요")
    tool = REPO / "tools/collection-runtime"
    group = Path("/proc/self/cgroup").read_bytes()
    pid_file = RUN / "collection-processes/pids"
    if pid_file.exists():
        for row in pid_file.read_text().splitlines():
            pid = int(row.split()[1])
            owner = Path(f"/proc/{pid}/cgroup")
            if owner.exists() and owner.read_bytes() != group:
                raise ValueError("기존 팔 프로세스 소유권 인계 필요")
    for command in ("start", "action-start"):
        subprocess.run([tool, command], check=True)
    # setsid children stay in this systemd cgroup. Reject stale PIDs from an earlier attempt.
    pids = pid_file.read_text().splitlines()
    if not any(row.startswith("manipulation_action ") for row in pids):
        raise ValueError("팔 Action 서버 기동 확인 필요")
    while True:
        for row in pids:
            pid = int(row.split()[1])
            if Path(f"/proc/{pid}/cgroup").read_bytes() != group:
                raise ValueError("팔 제어 프로세스 재확인 필요")
        write_status("collection", [])
        time.sleep(3)


def runtime_snapshot(node, client):
    from cleany_interfaces.srv import GetRuntimeSnapshot
    import rclpy

    if not client.service_is_ready():
        return {}
    future = client.call_async(GetRuntimeSnapshot.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=1)
    if not future.done():
        client.remove_pending_request(future)
        return {}
    return json.loads(future.result().snapshot_json)


def watch(role, proc=None):
    """Report observed readiness independently from process liveness."""
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from cleany_interfaces.srv import GetRuntimeSnapshot
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CameraInfo, LaserScan
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = rclpy.create_node("field_" + role + "_status")
    stamps = {}
    def received(topic, message):
        stamp = message.header.stamp
        age = (node.get_clock().now().nanoseconds - stamp.sec * 10**9 - stamp.nanosec) / 1e9
        if 0 <= age < 3:
            stamps[topic] = time.monotonic()
    inputs = {"rgbd": [(CameraInfo, "/camera/color/camera_info"), (CameraInfo, "/camera/depth/camera_info")],
              "navigation": [(LaserScan, "/scan"), (Odometry, "/odom")],
              "pose": [(PoseStamped, "/telemetry/pose")]}.get(role, [])
    for kind, topic in inputs:
        node.create_subscription(kind, topic, lambda msg, t=topic: received(t, msg), qos_profile_sensor_data)
    client = node.create_client(GetRuntimeSnapshot, "mission/snapshot") if role == "mission" else None
    deadline = 0.
    # TensorRT and USB initialization may take several minutes on a cold board.
    observation_deadline = time.monotonic() + 300
    try:
        while rclpy.ok() and (proc is None or proc.poll() is None):
            rclpy.spin_once(node, timeout_sec=.25)
            if time.monotonic() < deadline:
                continue
            deadline = time.monotonic() + 3
            reasons = [topic + " 최신 센서 표본 대기" for _, topic in inputs
                       if time.monotonic() - stamps.get(topic, float("-inf")) > 3]
            if role == "mission":
                snapshot = runtime_snapshot(node, client)
                if not snapshot:
                    reasons.append("미션 Runtime 응답 대기")
                elif not snapshot.get("ready") and not snapshot.get("active_request"):
                    reasons.append(str(snapshot.get("reason") or "실물 준비 조건 대기"))
            elif role == "perception":
                if "/perception/get_scene_snapshot" not in dict(node.get_service_names_and_types()):
                    reasons.append("장면 관측 서비스 대기")
            write_status(role, reasons)
            if role in ("rgbd", "perception"):
                if not reasons:
                    observation_deadline = time.monotonic() + 45
                elif time.monotonic() >= observation_deadline:
                    raise ValueError("센서·인식 응답 시간 초과, 재확인 대기")
        if rclpy.ok():
            raise RuntimeError(f"{role} child exited with {proc.returncode}")
    except (ExternalShutdownException, KeyboardInterrupt):
        return
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def assert_no_owner(role):
    import rclpy

    rclpy.init()
    node = rclpy.create_node("field_" + role + "_ownership")
    try:
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        names = set(node.get_node_names())
        owned = {"rgbd": {"camera", "calibrated_rgbd"}, "perception": {"perception_inspector"},
                 "collection": {"controller_manager", "manipulation_jetson_server"},
                 "navigation": {"hardware_runtime", "bt_navigator"},
                 "mission": {"mission_runtime", "control_bridge"},
                 "pose": {"pose_observer", "pose_backend_bridge"}}
        if owned[role] & names:
            raise ValueError("기존 실행 세션에서 장치·서비스 인계 필요")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def pose_ready():
    import rclpy
    from cleany_interfaces.srv import GetRuntimeSnapshot

    rclpy.init()
    node = rclpy.create_node("field_pose_check")
    try:
        client = node.create_client(GetRuntimeSnapshot, "mission/snapshot")
        client.wait_for_service(timeout_sec=2)
        snapshot = runtime_snapshot(node, client)
        if not snapshot or not (snapshot.get("ready") or snapshot.get("active_request")):
            raise ValueError("운영 지도 위치 정합 및 미션 준비 확인 대기")
        if snapshot.get("target_map") != bound_targets().binding:
            raise ValueError("실행 중인 미션 지도와 승인 좌석 파일 불일치")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def command(role):
    if role == "pose":
        pose_ready()
        return [str(REPO / "tools/ros2/dashboard-observe"), "pose"]
    if role == "rgbd":
        return ["ros2", "launch", "cleany_perception", "realsense.launch.py",
                "serial_no:=" + head_camera_serial(),
                "config_file:=" + str(REPO / "ros2_ws/src/cleany_perception/config/realsense_pregrasp.yaml"),
                "rgb_intrinsics_file:=" + str(CONFIG / "head-rgb-intrinsics.json")]
    if role == "perception":
        return [str(REPO / "tools/perception-native"), "enable_reference_observation:=true",
                "enable_wrist_observation:=true"]
    if role == "navigation":
        name = datetime.now().strftime("%Y-%m-%d_%H%M%S_field-boot")
        runs = Path(os.environ.get("CLEANY_FIELD_NAVIGATION_RUNS", "/var/opt/cleany/runs/navigation/integration"))
        return [str(REPO / "tools/ros2/run-navigation"), "--run-root", str(runs / name),
                "--state-root", str(STATE / "teleop"), "--profile", os.environ["CLEANY_BASE_PROFILE"],
                "--map", str(MAP), "--dds-profile", str(DDS), "--rmw", FIELD_RMW,
                "--mission-targets", str(TARGETS), "--mission-grid-revision", os.environ["CLEANY_MISSION_GRID_REVISION"]]
    from ament_index_python.packages import get_package_share_directory
    config = Path(get_package_share_directory("cleany_mission_manager")) / "config/runtime_jetson_desk.yaml"
    observations = Path(os.environ.get("CLEANY_OBSERVATION_DIRECTORY", dashboard_database().parent / "observations"))
    return ["ros2", "launch", "cleany_mission_manager", "mission_runtime.launch.py",
            "runtime_config:=" + str(config), "targets_file:=" + str(TARGETS),
            "collection_config:=" + str(COLLECTION), "observation_directory:=" + str(observations),
            "use_sim_time:=false", "navigation_backend:=mission_drive", "supervision_mode:=required",
            "restart_as_unit:=true", "post_mission:=return_home", "device_token_file:=" + str(TOKEN),
            "journal_path:=" + str(STATE / "desk-field-missions.db"),
            "bridge_journal_path:=" + str(STATE / "desk-field-bridge.db")]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("role", choices=ROLES)
    parser.add_argument("--arm-check", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--check", action="store_true", help="Check configuration without starting any service")
    parser.add_argument("--observe-native-sensors", action="store_true",
                        help="Monitor the RGB-D and perception services started by systemd")
    args = parser.parse_args()
    reasons = blockers(args.role)
    if args.check:
        print(json.dumps({"component": args.role, "blockers": reasons}, ensure_ascii=False))
        return int(bool(reasons))
    if args.arm_check:
        if args.role != "collection" or reasons:
            return 1
        try:
            check_arm_journals()
            safe_inputs()
        except (ValueError, sqlite3.Error) as error:
            write_status(args.role, [str(error)])
            return 1
        return 0
    write_status(args.role, reasons or ["서비스 기동 중"])
    if reasons:
        return 1
    os.environ.update(ROS_DOMAIN_ID="0", RMW_IMPLEMENTATION=FIELD_RMW, CLEANY_INSTALL_BASE=str(OVERLAY))
    if FIELD_RMW == "rmw_fastrtps_cpp":
        os.environ.pop("CYCLONEDDS_URI", None)
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(DDS)
    elif FIELD_RMW == "rmw_cyclonedds_cpp":
        os.environ.pop("FASTRTPS_DEFAULT_PROFILES_FILE", None)
        os.environ["CYCLONEDDS_URI"] = DDS.as_uri()
    else:
        raise ValueError(f"Unsupported field RMW implementation: {FIELD_RMW}")
    try:
        if args.observe_native_sensors and args.role in ("rgbd", "perception"):
            watch(args.role)
            return 0
        else:
            assert_no_owner(args.role)
            if args.role == "collection":
                collection()
            else:
                proc = subprocess.Popen(command(args.role))
                watch(args.role, proc)
    except ValueError as error:
        write_status(args.role, [str(error)])
    except (OSError, RuntimeError, subprocess.SubprocessError, sqlite3.Error) as error:
        print(error, file=sys.stderr, flush=True)
        write_status(args.role, ["기동 실패 · 서비스 로그 확인 필요"])
    return 1


if __name__ == "__main__":
    sys.exit(main())
