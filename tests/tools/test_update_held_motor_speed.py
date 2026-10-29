"""Speed maintenance must never change a goal, torque or EEPROM."""
import pytest

from validation.update_held_motor_speed import update


class Bus:
    def __init__(self):
        self.data = {(motor, address): value for motor in range(1, 13)
                     for address, value in {3: (2825 if (motor - 1) % 6 in (1, 2) else 777),
                         33: 0, 40: 1, 41: 254, 42: 2000, 44: 0, 46: 800,
                         48: 800, 56: 1997, 58: 0, 63: 30, 65: 0, 13: 80}.items()}
        self.writes = []
        self.fail_motor = None

    def read(self, motor, address, size):
        return self.data[motor, address]

    def write(self, motor, address, size, value):
        self.writes.append((motor, address, size, value))
        self.data[motor, address] = value
        if motor == self.fail_motor and value == 1304:
            raise RuntimeError('lost acknowledgement')


def test_updates_only_speed_and_preserves_existing_hold():
    bus = Bus()
    result = update(bus, previous=800, requested=1304)
    assert bus.writes == [(motor, 46, 2, 1304) for motor in range(1, 13)]
    assert all(state['goal'] == 2000 and state['torque'] == 1 for state in result['after'].values())


@pytest.mark.parametrize('address,value', [(40, 0), (58, 3), (42, 2200), (65, 1), (46, 999)])
def test_preflights_last_motor_before_any_write(address, value):
    bus = Bus()
    bus.data[12, address] = value
    with pytest.raises(RuntimeError):
        update(bus, previous=800, requested=1304)
    assert bus.writes == []


def test_rolls_back_including_uncertain_write():
    bus = Bus()
    bus.fail_motor = 7
    with pytest.raises(RuntimeError, match=r'rollback failures=\[\]'):
        update(bus, previous=800, requested=1304)
    assert all(bus.data[motor, 46] == 800 for motor in range(1, 13))
    assert all(address == 46 and size == 2 for _, address, size, _ in bus.writes)
