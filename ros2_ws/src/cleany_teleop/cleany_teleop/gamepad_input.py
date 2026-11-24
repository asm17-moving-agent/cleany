"""Publish timestamped XInput reports from the LeadJoy receiver as Joy messages."""

from pathlib import Path
import time

from evdev import InputDevice, ecodes as ec
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from cleany_teleop.gamepad import StickSnapbackFilter

FRAME = "leadjoy_xeno_plus"
STICK_CENTER_BAND = 0.05
AXES = (ec.ABS_X, ec.ABS_Y, ec.ABS_Z, ec.ABS_RX, ec.ABS_RY, ec.ABS_RZ)
BUTTONS = (
    ec.BTN_A,
    ec.BTN_B,
    ec.BTN_X,
    ec.BTN_Y,
    ec.BTN_TL,
    ec.BTN_TR,
    ec.BTN_SELECT,
    ec.BTN_START,
    ec.BTN_MODE,
    ec.BTN_THUMBL,
    ec.BTN_THUMBR,
)


def receiver_path(path: str, sys_input: Path = Path("/sys/class/input")) -> str:
    """Select a device automatically only when exactly one xpad event device matches."""
    if path != "auto":
        return path
    matches = []
    for event in sys_input.glob("event*"):
        try:
            if (event / "device/name").read_text().strip() == "LeadJoy Xeno Plus":
                matches.append(f"/dev/input/{event.name}")
        except FileNotFoundError:
            continue  # The sysfs entry can change as the device connects or disconnects.
    if not matches:
        raise FileNotFoundError("LeadJoy Xeno Plus 이벤트 장치를 기다립니다")
    if len(matches) > 1:
        raise ValueError("LeadJoy Xeno Plus가 여러 대입니다. device에 장치 경로를 지정하세요")
    return matches[0]


class GamepadInput(Node):
    """Publish complete local receiver reports on /joy."""

    def __init__(self):
        super().__init__("gamepad_input")
        self.path = self.declare_parameter("device", "auto").value
        self.output = self.create_publisher(Joy, "joy", 10)
        self.device = None
        self.next_open = self.last_publish = 0.0
        self.last_buttons = None
        self.left_stick = StickSnapbackFilter(STICK_CENTER_BAND)
        self.right_stick = StickSnapbackFilter(STICK_CENTER_BAND)
        self.create_timer(0.005, self.poll)

    def connect(self):
        dev = InputDevice(receiver_path(self.path))
        try:
            if (dev.info.vendor, dev.info.product) != (0x4131, 0x3519):
                raise ValueError("LeadJoy Xeno Plus 수신기 식별자가 다릅니다")
            if ec.MSC_TIMESTAMP not in dev.capabilities().get(ec.EV_MSC, ()):
                raise ValueError("실제 보고서 시각을 제공하는 cleany_xpad가 필요합니다")
            self.axes = {code: dev.absinfo(code).value for code in AXES}
            self.keys = set(dev.active_keys())
        except BaseException:
            dev.close()
            raise
        self.device = dev
        self.report = False
        self.last_report = None
        self.last_buttons = None
        self.get_logger().info(f"패드 입력 연결: {dev.path}")

    def publish(self, stamp_ns, axes=(), buttons=()):
        message = Joy()
        message.header.frame_id = FRAME
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 10**9)
        message.axes = list(axes)
        message.buttons = list(buttons)
        self.output.publish(message)

    def disconnect(self):
        if self.device:
            self.device.close()
            self.device = None
            self.publish(time.time_ns())
        self.next_open = time.monotonic() + 0.5

    def poll(self):
        if self.device is None:
            if time.monotonic() >= self.next_open:
                try:
                    self.connect()
                except (OSError, ValueError) as exc:
                    self.get_logger().warning(str(exc), throttle_duration_sec=5)
                    self.disconnect()
            return
        try:
            for event in self.device.read():
                if event.type == ec.EV_SYN and event.code == ec.SYN_DROPPED:
                    self.disconnect()
                    return
                if event.type == ec.EV_ABS and event.code in self.axes:
                    self.axes[event.code] = event.value
                elif event.type == ec.EV_KEY:
                    if event.value:
                        self.keys.add(event.code)
                    else:
                        self.keys.discard(event.code)
                elif event.type == ec.EV_MSC and event.code == ec.MSC_TIMESTAMP:
                    self.report = True
                elif (
                    event.type == ec.EV_SYN
                    and event.code == ec.SYN_REPORT
                    and self.report
                ):
                    self.report = False
                    stamp_ns = event.sec * 10**9 + event.usec * 1000
                    if self.last_report is None or not 0 <= stamp_ns - self.last_report < 150_000_000:
                        self.left_stick.reset()
                        self.right_stick.reset()
                    self.last_report = stamp_ns
                    # Positive axes correspond to forward, left and counterclockwise.
                    axes = [
                        (
                            self.axes[c] / 255.0
                            if c in (ec.ABS_Z, ec.ABS_RZ)
                            else -self.axes[c]
                            / (32768.0 if self.axes[c] < 0 else 32767.0)
                        )
                        for c in AXES
                    ]
                    # The stick can cross neutral between two 20 ms publications.
                    raw_left, raw_right = axes[:2], axes[3:5]
                    stamp = stamp_ns / 1e9
                    axes[:2] = self.left_stick.apply(raw_left, stamp)
                    axes[3:5] = self.right_stick.apply(raw_right, stamp)
                    buttons = tuple(int(c in self.keys) for c in BUTTONS)
                    if (buttons == self.last_buttons and not self.left_stick.released
                            and not self.right_stick.released
                            and 0 <= stamp_ns - self.last_publish < 20_000_000):
                        continue
                    self.last_publish, self.last_buttons = stamp_ns, buttons
                    axes.extend(raw_left + raw_right)
                    self.publish(stamp_ns, axes, buttons)
        except BlockingIOError:
            pass
        except OSError:
            self.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = GamepadInput()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.device:
            node.device.close()
        node.destroy_node()
        rclpy.try_shutdown()
