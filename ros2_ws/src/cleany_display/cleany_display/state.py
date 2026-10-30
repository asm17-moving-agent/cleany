"""Select a display expression from observed ROS state, without commanding motion."""

from dataclasses import dataclass
import math
from typing import Any

MODES = frozenset(('idle', 'moving', 'checking', 'collecting', 'complete', 'charging', 'help'))


@dataclass
class Sample:
    data: dict[str, Any]
    received: float
    changed: float


class DisplayState:
    """Retain event-driven state while its publisher exists; expire sensor samples."""

    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config
        self.samples: dict[str, Sample] = {}
        self.ever_connected = False
        self.last_connected = 0.0
        self.last_running_mission: str | None = None

    def ingest(self, source: str, data: dict[str, Any], now: float) -> None:
        data = dict(data)
        if source == 'mission' and data.get('state') not in ('IDLE', 'ERROR') and data.get('mission_id'):
            self.last_running_mission = data['mission_id']
        elif source == 'result':
            data['_observed_running'] = bool(self.last_running_mission) and (
                data.get('mission_id') == self.last_running_mission)
        previous = self.samples.get(source)
        if source == 'manipulation' and previous:
            old_stamp = previous.data.get('updated_at_ns', 0)
            if data.get('updated_at_ns', 0) < old_stamp:
                return
        changed = previous.changed if previous and previous.data == data else now
        self.samples[source] = Sample(data, now, changed)

    def override(self, data: dict[str, Any], now: float) -> bool:
        """Accept a bounded, display-only override; malformed input has no effect."""
        if not isinstance(data.get('mode'), str) or data['mode'] not in MODES:
            return False
        duration = data.get('duration_seconds', 5.0)
        if isinstance(duration, bool) or not isinstance(duration, (int, float)) or not math.isfinite(duration):
            return False
        if not 0 < duration <= self.config['override_max_seconds']:
            return False
        direction = data.get('direction', 'forward')
        if direction not in ('left', 'forward', 'right'):
            return False
        clean = {'mode': data['mode'], 'direction': direction, 'duration_seconds': duration}
        if isinstance(data.get('message'), str):
            clean['message'] = data['message'][:120]
        self.samples['display'] = Sample(clean, now, now)
        return True

    def snapshot(self, now: float, publishers: set[str], wall_time: float) -> dict[str, Any]:
        c = self.config
        active: dict[str, Sample] = {}
        for source, sample in self.samples.items():
            if source == 'display':
                if now - sample.received < sample.data['duration_seconds']:
                    active[source] = sample
            elif source in publishers:
                ttl = c.get(f'{source}_timeout_seconds', math.inf)
                if now - sample.received <= ttl:
                    active[source] = sample
        if any(source != 'display' for source in active):
            self.ever_connected = True
            self.last_connected = now
        output: dict[str, Any] = dict(mode='idle', direction='forward', battery_percent=None,
                                     collected_count=None, message='', source='waiting',
                                     connected=bool(active), sources=sorted(active))
        if not active:
            if self.ever_connected and now - self.last_connected > c['disconnect_grace_seconds']:
                output.update(mode='help', source='disconnected', message=c['connection_message'])
            return output

        def choose(mode: str, source: str, message: str = '') -> dict[str, Any]:
            output.update(mode=mode, source=source, message=message)
            return output

        mission = active.get('mission')
        manip = active.get('manipulation')
        battery = active.get('battery')
        if battery:
            percentage = battery.data.get('percentage')
            if isinstance(percentage, (int, float)) and math.isfinite(percentage) and 0 <= percentage <= 1:
                output['battery_percent'] = round(percentage * 100)
        if mission and mission.data.get('state') == 'ERROR':
            return choose('help', 'mission', c['help_message'])
        if manip and (manip.data.get('human_confirmation_required') or
                      manip.data.get('record_state') in ('INTERRUPTED', 'RECORDING_FAILED')):
            return choose('help', 'manipulation', c['help_message'])
        if 'display' in active:
            data = active['display'].data
            output['direction'] = data['direction']
            return choose(data['mode'], 'display', data.get('message', ''))
        if battery and battery.data.get('power_supply_status') == 1:
            return choose('charging', 'battery')
        if manip and manip.data.get('record_state') == 'ACTIVE':
            checking = manip.data.get('substage') in c['checking_substages']
            return choose('checking' if checking else 'collecting', 'manipulation')
        if mission:
            mode = c['mission_modes'].get(mission.data.get('state'), 'help')
            if mission.data.get('state') == 'WORKING':
                mode = c['working_stage_modes'].get(mission.data.get('bt_stage'), 'checking')
            if mode not in ('idle', 'complete'):
                return choose(mode, 'mission', c['help_message'] if mode == 'help' else '')
        odometry = active.get('odometry')
        if odometry:
            linear = odometry.data.get('linear', 0.0)
            angular = odometry.data.get('angular', 0.0)
            if math.isfinite(linear) and math.isfinite(angular) and (
                    abs(linear) > c['linear_threshold_m_s'] or abs(angular) > c['angular_threshold_rad_s']):
                output['direction'] = ('left' if angular > c['angular_threshold_rad_s'] else
                                       'right' if angular < -c['angular_threshold_rad_s'] else 'forward')
                return choose('moving', 'odometry')
        # Durable success records must not replay a celebration on every restart.
        if manip and manip.data.get('has_result'):
            age = max(now - manip.changed, wall_time - manip.data.get('updated_at_ns', 0) / 1e9)
            status = manip.data.get('status')
            if status == 'SUCCESS' and age < c['completion_seconds']:
                return choose('complete', 'manipulation', '수거 완료!')
            if status in ('FAILED', 'BLOCKED', 'FATAL') and age < c['failure_seconds']:
                return choose('help', 'manipulation', c['help_message'])
        # MissionResult has no timestamp: only celebrate when this process observed its mission running.
        result = active.get('result')
        if result and result.data.get('_observed_running') and now - result.changed < c['completion_seconds']:
            if result.data.get('needs_human_review') or result.data.get('outcome') not in ('SUCCESS', 'PARTIAL_SUCCESS'):
                return choose('help', 'result', c['help_message'])
            output['collected_count'] = len(result.data.get('completed_tasks', []))
            return choose('complete', 'result')
        if mission and mission.data.get('state') == 'POST_MISSION' and now - mission.changed < c['completion_seconds']:
            return choose('complete', 'mission')
        return output
