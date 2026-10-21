"""Read-only serial observations shared by TCP measurement tools."""
import fcntl
import termios
import time


class JointReader:
    """Only READ instructions; no connect/configure or torque writes."""

    def __init__(self, config: dict):
        from scservo_sdk import PacketHandler, PortHandler
        self.port = PortHandler(config['serial_port'])
        self.packet = PacketHandler(0)
        self.ids = config['motor_ids']
        self.locked = False
        try:
            if not self.port.setBaudRate(config['baudrate']):
                raise RuntimeError('Cannot open head motor port')
            fcntl.flock(self.port.ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(self.port.ser.fileno(), termios.TIOCEXCL)
            self.locked = True
        except Exception:
            self.close()
            raise

    def read(self) -> dict:
        result = {'host_monotonic_start_ns': time.monotonic_ns(), 'joints': {}}
        for name, motor in self.ids.items():
            position, status, error = self.packet.read2ByteTxRx(self.port, motor, 56)
            if status or error:
                raise RuntimeError(f'{name} ID {motor}: comm={status}, error={error}')
            result['joints'][name] = {'id': motor, 'raw_tick': position}
        result['host_monotonic_end_ns'] = time.monotonic_ns()
        return result

    def close(self) -> None:
        if self.locked:
            fcntl.ioctl(self.port.ser.fileno(), termios.TIOCNXCL)
            self.locked = False
        self.port.closePort()
