"""제공된 VL53L7CH ESP32 JSONL 펌웨어의 native USB 브리지입니다."""
import time
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
from cleany_interfaces.msg import TofFrame
from .ros_support import steady_timer
from .transport import Stream


class TofDriver(Node):
    """tof/frame(TofFrame, reliable, depth 1)을 발행하며 무효 데이터를 대체하지 않습니다.

    serial 오류 시 포트를 닫고 새로운 stop/config/start 연결 절차를 강제합니다.
    steady timer로 재시도를 제한하며 장치 시각으로 오래된 USB 큐를 거부합니다.
    """
    def __init__(self, **kwargs):
        super().__init__('tof_driver', **kwargs)
        self.declare_parameter('port', '/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_8C:BF:EA:85:0F:00-if00')
        self.pub = self.create_publisher(TofFrame, 'tof/frame', QoSProfile(depth=1))
        self.port = None
        self.retry = 0.
        self.stream = Stream()
        self.state = 'closed'
        self.deadline = 0.
        self.last_frame = 0.
        steady_timer(self, .01, self.tick)

    def close(self):
        if self.port:
            # 시간 초과를 기다리기 전에 아직 최신인 캐시 프레임을 무효화합니다.
            msg = TofFrame()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'tof_link'
            self.pub.publish(msg)
            self.port.close()
        self.port = None
        self.state = 'closed'
        self.retry = time.monotonic()+1.

    def tick(self):
        now = time.monotonic()
        try:
            if self.port is None:
                if now < self.retry:
                    return
                self.port = serial.Serial(self.get_parameter('port').value, 115200, timeout=0, write_timeout=.1)
                self.port.reset_input_buffer()
                self.stream = Stream()
                self.port.write(b'\nstop\n')
                self.state, self.deadline = 'stopping', now+5.
            if self.port.in_waiting > 65536:
                raise ValueError('USB receiver backlog')
            records = self.stream.feed(self.port.read(min(65536, self.port.in_waiting)))
            for record in records:
                if self.state == 'stopping':
                    if record.get('event') == 'stopped':
                        self.port.reset_input_buffer()
                        self.stream = Stream()
                        self.port.write(b'cnh off\nconfig 64 15 400000\nstart\n')
                        self.state, self.deadline = 'starting', now+15.
                        break  # 이번 읽기에서 session 이전 프레임은 모두 버립니다.
                elif self.state == 'starting':
                    if record.get('type') == 'info' and record.get('state') == 'running':
                        self.state, self.last_frame = 'running', now
                elif record.get('type') != 'frame' and (record.get('state', 'running') != 'running'
                        or record.get('status', 0) != 0):
                    raise ValueError('firmware unavailable: '+str(record.get('event', record.get('state'))))
                elif record.get('type') == 'frame':
                    acquisition = self.stream.frame(record, now)
                    msg = TofFrame()
                    stamp = self.get_clock().now().nanoseconds-int((now-acquisition)*1e9)
                    if stamp < 0:
                        raise ValueError('ROS clock predates acquisition')
                    msg.header.stamp = Time(nanoseconds=stamp).to_msg()
                    msg.header.frame_id = 'tof_link'
                    msg.sensor_ready = True
                    msg.sequence, msg.device_time_us = record['seq'], record['time_us']
                    msg.distance_m = [float(x)/1000 for x in record['distance_mm']]
                    msg.sigma_m = [float(x)/1000 for x in record['sigma_mm']]
                    msg.target_status, msg.target_count, msg.valid = (record[k] for k in ('target_status', 'target_count', 'valid'))
                    self.pub.publish(msg)
                    self.last_frame = now
            if (self.state in ('starting', 'stopping') and now > self.deadline
                    or self.state == 'running' and now-self.last_frame > .3):
                raise ValueError('sensor session timed out')
        except (serial.SerialException, OSError, ValueError, TypeError, KeyError) as exc:
            self.get_logger().warning(str(exc))
            self.close()

    def destroy_node(self):
        self.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = TofDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
