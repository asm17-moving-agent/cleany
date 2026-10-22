"""Physical collection configuration and evidence gates, independent of ROS."""
from dataclasses import dataclass
import math
from .physical_grasp_selection import ARM_JOINT_NAMES
from .height_correction import HeightCorrectionPolicy

RETURN_TO_PICK = 'return_to_pick'


def temporary_return_mode(config: dict) -> bool:
    return config.get('operation_mode') == 'pick_return_release'


@dataclass(frozen=True)
class PickupReturnPolicy:
    height_offset_m: float
    height_tolerance_m: float
    reverse_enabled: bool
    check_collisions: bool
    correction_max_attempts: int
    correction_max_step_m: float

    @classmethod
    def from_config(cls, config: dict) -> 'PickupReturnPolicy':
        offset = config.get('return_to_pick_height_offset_m', 0.)
        tolerance = config.get('return_to_pick_height_tolerance_m', .01)
        for name, value in (('offset', offset), ('tolerance', tolerance)):
            if (type(value) not in (int, float) or not math.isfinite(value)
                    or value < 0 or (name == 'tolerance' and value == 0)):
                raise ValueError(f'return_to_pick_height_{name}_m is invalid')
        if offset > 0 and tolerance >= offset:
            raise ValueError('return_to_pick_height_tolerance_m must be smaller than positive offset')
        reverse = config.get('return_to_pick_reverse_enabled', False)
        collisions = config.get('return_to_pick_collision_check_enabled', True)
        for key, value in (('reverse_enabled', reverse), ('collision_check_enabled', collisions)):
            if type(value) is not bool:
                raise ValueError(f'return_to_pick_{key} must be boolean')
        if reverse and (not temporary_return_mode(config) or offset <= 0):
            raise ValueError('return_to_pick_reverse_enabled requires temporary return with positive height offset')
        if not collisions and not reverse:
            raise ValueError('return_to_pick_collision_check_enabled=false requires reverse return')
        attempts = config.get('return_to_pick_height_correction_max_attempts', 2)
        step = config.get('return_to_pick_height_correction_max_step_m', .005)
        if type(attempts) is not int or not 0 <= attempts <= 4:
            raise ValueError('return_to_pick_height_correction_max_attempts must be 0..4')
        if type(step) not in (int,float) or not math.isfinite(step) or not 0 < step <= .01:
            raise ValueError('return_to_pick_height_correction_max_step_m must be positive and at most 0.01')
        return cls(float(offset), float(tolerance), reverse, collisions, attempts, float(step))


@dataclass(frozen=True)
class ArmCollectionConfig:
    closed: float
    opened: float
    width: float
    home: tuple[float, ...] | None
    bins: dict[str, tuple[float, ...]]

    def opening(self, width: float) -> float:
        if not math.isfinite(width) or not 0 <= width <= self.width:
            raise ValueError('Object opening outside measured gripper range')
        return self.closed+(self.opened-self.closed)*width/self.width


def contact_verification_required(config: dict) -> bool:
    """Only an explicit temporary skip bypasses motor/identity evidence."""
    return not (temporary_return_mode(config)
                and config.get('temporary_contact_verification') == 'skip')


def grasp_identity_verification_required(config: dict) -> bool:
    """Only temporary return trials may retain motor contact without wrist identity."""
    return contact_verification_required(config) and not (
        temporary_return_mode(config) and config.get('wrist_observation_enabled', True) is False)


def configuration_issues(config: dict) -> list[str]:
    if not isinstance(config, dict):
        return ['configuration must be an object']
    issues = []
    try:
        PickupReturnPolicy.from_config(config)
    except ValueError as error:
        issues.append(str(error))
    try:
        HeightCorrectionPolicy.from_config(config)
    except ValueError as error:
        issues.append(str(error))
    mode = config.get('operation_mode', 'full_collection')
    if mode not in ('full_collection', 'grasp_lift_hold', 'grasp_lift_return_hold', 'pick_return_release'):
        issues.append('unknown operation_mode')
    full_collection = mode == 'full_collection'
    if temporary_return_mode(config):
        if config.get('temporary_contact_verification') not in ('skip', 'raw_feedback'):
            issues.append('temporary return requires explicit skip or raw_feedback contact verification')
        if config.get('temporary_placement_verification') != 'assume_success':
            issues.append('temporary return requires explicit assumed placement')
        if config.get('label_destinations') != {'*': RETURN_TO_PICK}:
            issues.append('temporary return destination must be return_to_pick for all labels')
    if 'continuous_scene_updates' in config and type(config['continuous_scene_updates']) is not bool:
        issues.append('continuous_scene_updates must be boolean')
    if config.get('frame') != 'base_link':
        issues.append('frame must be base_link')
    if config.get('commissioning_verified') is not True:
        issues.append('physical motion/stop commissioning is not verified')
    arms = config.get('arms', {})
    if not arms or not isinstance(arms, dict):
        return issues+['no configured arms']
    destinations = config.get('label_destinations')
    if full_collection and (not isinstance(destinations, dict) or not destinations or not all(
            isinstance(k,str) and k and isinstance(v,str) and v for k,v in destinations.items())):
        issues.append('label-to-bin destinations are missing/invalid')
        destinations = {}
    elif not isinstance(destinations, dict):
        destinations = {}
    for name, raw in arms.items():
        if name not in ARM_JOINT_NAMES:
            issues.append('unknown arm '+name)
            continue
        if not isinstance(raw,dict):
            issues.append(f'{name}: arm configuration must be an object')
            continue
        if raw.get('home_joint_rad') is None:
            issues.append(f'{name}: home joint positions are missing')
        if full_collection and not raw.get('bins'):
            issues.append(f'{name}: bin joint positions are missing')
        try:
            arm = load_arm(raw, require_bins=full_collection)
            if full_collection:
                for destination in destinations.values():
                    if destination not in arm.bins:
                        issues.append(f'{name}: missing bin {destination}')
        except (KeyError, TypeError, ValueError) as error:
            issues.append(f'{name}: {error}')
    for key in ('lift_height_m', 'cartesian_step_m', 'cartesian_tolerance_m', 'evidence_max_age_sec',
                'gripper_close_step_rad', 'pregrasp_opening_margin_m'):
        value = config.get(key)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            issues.append('missing/invalid '+key)
    count = config.get('empty_confirmations')
    if type(count) is not int or count < 2:
        issues.append('empty_confirmations must be at least 2')
    step = config.get('gripper_close_step_rad')
    if isinstance(step,(int,float)) and step > .005:
        issues.append('gripper_close_step_rad exceeds 0.005')
    if type(config.get('maximum_cycles')) is not int or config['maximum_cycles'] < 1:
        issues.append('maximum_cycles must be a positive integer')
    visual = config.get('visual_servo_enabled', False)
    if type(visual) is not bool:
        issues.append('visual_servo_enabled must be boolean')
    pregrasp_motion = config.get('pregrasp_motion_enabled', True)
    if type(pregrasp_motion) is not bool:
        issues.append('pregrasp_motion_enabled must be boolean')
    elif not pregrasp_motion:
        for key in ('visual_servo_enabled', 'wrist_observation_enabled'):
            if config.get(key, key == 'wrist_observation_enabled') is not False:
                issues.append('pregrasp_motion_enabled=false requires '+key+'=false')
    if type(config.get('direct_grasp_shortcut_enabled', False)) is not bool:
        issues.append('direct_grasp_shortcut_enabled must be boolean')
    if type(config.get('direct_grasp_pose_checks_enabled', True)) is not bool:
        issues.append('direct_grasp_pose_checks_enabled must be boolean')
    if type(config.get('grasp_collision_check_enabled', True)) is not bool:
        issues.append('grasp_collision_check_enabled must be boolean')
    if type(config.get('gripper_opening_collision_check_enabled', True)) is not bool:
        issues.append('gripper_opening_collision_check_enabled must be boolean')
    if type(config.get('lift_collision_check_enabled', True)) is not bool:
        issues.append('lift_collision_check_enabled must be boolean')
    if type(config.get('retreat_after_release_collision_check_enabled', True)) is not bool:
        issues.append('retreat_after_release_collision_check_enabled must be boolean')
    if type(config.get('retreat_reverse_approach_enabled', False)) is not bool:
        issues.append('retreat_reverse_approach_enabled must be boolean')
    if type(config.get('held_object_octomap_collision_check_enabled', True)) is not bool:
        issues.append('held_object_octomap_collision_check_enabled must be boolean')
    wrist_observation = config.get('wrist_observation_enabled', True)
    if type(wrist_observation) is not bool:
        issues.append('wrist_observation_enabled must be boolean')
    stationary_target = config.get('assume_stationary_target', False)
    if type(stationary_target) is not bool:
        issues.append('assume_stationary_target must be boolean')
    elif stationary_target:
        # Explicit supervised trial assumption, not observed target stability.
        if not temporary_return_mode(config):
            issues.append('assume_stationary_target requires temporary pick_return_release mode')
        if visual is not False:
            issues.append('assume_stationary_target requires visual_servo_enabled=false')
        if wrist_observation is not False:
            issues.append('assume_stationary_target requires wrist_observation_enabled=false')
    approach_replanning = config.get('approach_collision_replanning_enabled', False)
    execution_replanning = config.get('approach_execution_replanning_enabled', False)
    if type(execution_replanning) is not bool:
        issues.append('approach_execution_replanning_enabled must be boolean')
    elif execution_replanning:
        if pregrasp_motion is not True:
            issues.append('approach_execution_replanning_enabled requires pregrasp_motion_enabled=true')
        if visual is not False or approach_replanning is not False:
            issues.append('approach_execution_replanning_enabled requires visual_servo_enabled=false '
                          'and approach_collision_replanning_enabled=false')
        attempts = config.get('approach_execution_replanning_attempts', 2)
        if type(attempts) is not int or not 1 <= attempts <= 2:
            issues.append('approach_execution_replanning_attempts must be between 1 and 2')
        reselections = config.get('approach_candidate_reselection_attempts', 2)
        if type(reselections) is not int or not 0 <= reselections <= 4:
            issues.append('approach_candidate_reselection_attempts must be between 0 and 4')
        timeout = config.get('approach_candidate_selection_timeout_sec', 60.)
        if type(timeout) not in (int, float) or not math.isfinite(timeout) or timeout <= 0:
            issues.append('invalid approach_candidate_selection_timeout_sec')
    approach_collision_check = config.get('approach_collision_check_enabled', True)
    if type(approach_collision_check) is not bool:
        issues.append('approach_collision_check_enabled must be boolean')
    elif not approach_collision_check:
        for key in ('approach_collision_replanning_enabled', 'fixed_jaw_clearance_enabled',
                    'visual_servo_enabled'):
            if config.get(key, False):
                issues.append('approach_collision_check_enabled=false requires '+key+'=false')
    fixed_jaw_clearance = config.get('fixed_jaw_clearance_enabled', False)
    if type(fixed_jaw_clearance) is not bool:
        issues.append('fixed_jaw_clearance_enabled must be boolean')
    elif fixed_jaw_clearance:
        meshes = config.get('fixed_jaw_collision_meshes')
        if (not isinstance(meshes, list) or not meshes
                or any(not isinstance(name, str) or not name or '/' in name or '\\' in name
                       for name in meshes)):
            issues.append('fixed_jaw_collision_meshes must contain collision mesh basenames')
    if type(approach_replanning) is not bool:
        issues.append('approach_collision_replanning_enabled must be boolean')
    elif approach_replanning:
        for key in ('approach_collision_detour_offset_m', 'approach_collision_final_straight_m',
                    'approach_collision_max_extra_length_m'):
            value = config.get(key)
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                issues.append('missing/invalid '+key)
        attempts = config.get('approach_collision_detour_attempts')
        if type(attempts) is not int or not 1 <= attempts <= 4:
            issues.append('approach_collision_detour_attempts must be between 1 and 4')
        refreshes = config.get('approach_collision_scene_refresh_attempts')
        if type(refreshes) is not int or not 0 <= refreshes <= 2:
            issues.append('approach_collision_scene_refresh_attempts must be between 0 and 2')
    if visual:
        for key in ('visual_approach_step_m',
                    'visual_servo_maximum_correction_m'):
            value = config.get(key)
            if (not isinstance(value, (int, float)) or not math.isfinite(value)
                    or value <= 0):
                issues.append('missing/invalid '+key)
        steps = config.get('visual_approach_max_steps')
        if type(steps) is not int or steps < 1:
            issues.append('visual_approach_max_steps must be a positive integer')
        if 'visual_approach_segment_m' in config:
            segment = config['visual_approach_segment_m']
            if (type(segment) not in (int, float) or not math.isfinite(segment)
                    or segment <= 0):
                issues.append('invalid visual_approach_segment_m')
            elif (type(config.get('cartesian_step_m')) in (int, float)
                  and segment < config['cartesian_step_m']):
                issues.append('visual_approach_segment_m is smaller than path resolution')
        for key in (
            'visual_servo_hard_correction_limit_m',
            'visual_observation_increment_limit_m',
            'visual_observation_consistency_m',
            'visual_grasp_capture_depth_m',
            'visual_grasp_capture_lateral_tolerance_m',
        ):
            if key not in config:
                continue
            value = config[key]
            if (type(value) not in (int, float) or not math.isfinite(value)
                    or value <= 0):
                issues.append('invalid '+key)
        hard_limit = config.get('visual_servo_hard_correction_limit_m')
        soft_limit = config.get('visual_servo_maximum_correction_m')
        if (type(hard_limit) in (int, float)
                and type(soft_limit) in (int, float)
                and hard_limit < soft_limit):
            issues.append(
                'visual_servo_hard_correction_limit_m is smaller than soft limit')
        extent_ratio = config.get('visual_observation_extent_ratio_limit')
        if (extent_ratio is not None
                and (type(extent_ratio) not in (int, float)
                     or not math.isfinite(extent_ratio)
                     or extent_ratio <= 1)):
            issues.append('invalid visual_observation_extent_ratio_limit')
        confirmation_frames = config.get('visual_observation_confirmation_frames')
        if (confirmation_frames is not None
                and (type(confirmation_frames) is not int
                     or confirmation_frames < 3
                     or confirmation_frames % 2 == 0)):
            issues.append(
                'visual_observation_confirmation_frames must be an odd integer at least 3')
        wrist_fallback = config.get(
            'visual_wrist_occlusion_fallback_enabled', False)
        if type(wrist_fallback) is not bool:
            issues.append(
                'visual_wrist_occlusion_fallback_enabled must be boolean')
        elif wrist_fallback and wrist_observation is not True:
            issues.append(
                'visual wrist fallback requires wrist observation')
        fallback_steps = config.get('visual_wrist_occlusion_max_steps')
        if (wrist_fallback is True
                and (type(fallback_steps) is not int or fallback_steps < 1)):
            issues.append(
                'visual_wrist_occlusion_max_steps must be a positive integer')
        if wrist_fallback is True:
            for key in ('visual_wrist_occlusion_entry_distance_m',
                        'visual_wrist_occlusion_max_distance_m',
                        'visual_wrist_occlusion_step_m',
                        'visual_wrist_occlusion_max_age_sec'):
                value = config.get(key)
                if (type(value) not in (int, float) or not math.isfinite(value)
                        or value <= 0):
                    issues.append('missing/invalid '+key)
            entry = config.get('visual_wrist_occlusion_entry_distance_m')
            distance = config.get('visual_wrist_occlusion_max_distance_m')
            fallback_step = config.get('visual_wrist_occlusion_step_m')
            if all(type(value) in (int, float) and math.isfinite(value) and value > 0
                   for value in (entry, distance, fallback_step)):
                if not fallback_step <= distance <= entry:
                    issues.append('wrist fallback requires step <= total distance <= entry distance')
    return list(dict.fromkeys(issues))


def load_arm(raw: dict, *, require_bins: bool = True) -> ArmCollectionConfig:
    grip = raw['gripper']
    if (grip.get('measured') is not True
            and grip.get('approved_existing_mapping') is not True):
        raise ValueError('approved gripper width/joint mapping is missing')
    values = [float(grip[k]) for k in ('closed_joint_rad', 'open_joint_rad', 'maximum_width_m')]
    if not all(math.isfinite(v) for v in values) or values[0] == values[1] or not 0 < values[2] <= .2:
        raise ValueError('invalid gripper calibration')
    def pose(value):
        if not isinstance(value, list) or len(value) != 5 or not all(isinstance(v,(float,int)) and math.isfinite(v) for v in value):
            raise ValueError('home/bin requires five measured arm joint positions')
        return tuple(float(v) for v in value)
    home_value = raw.get('home_joint_rad')
    home = None if home_value == 'startup' else pose(home_value)
    if require_bins and not raw.get('bins'):
        raise ValueError('bin joint positions are missing')
    bins = {k:pose(v) for k,v in raw.get('bins', {}).items()}
    return ArmCollectionConfig(*values, home, bins)


def check_evidence_identity(response, request, now_ns: int, max_age: float) -> None:
    stamp = response.header.stamp.sec*10**9+response.header.stamp.nanosec
    if (not response.success or response.arm != request.arm
            or response.reference_id != request.reference_id
            or getattr(response, 'wrist_reference_id', '') != getattr(request, 'wrist_reference_id', '')
            or response.source_snapshot_id != request.source_snapshot_id
            or response.source_object_id != request.source_object_id
            or not request.after_stamp_ns < stamp <= now_ns
            or now_ns-stamp > max_age*1e9):
        raise RuntimeError('Missing, stale, or wrong-object manipulation evidence')
    if (getattr(request, 'destination_id', '')
            and getattr(response, 'destination_id', '') != request.destination_id):
        raise RuntimeError('Manipulation evidence belongs to another destination')


def check_evidence(response, request, now_ns: int, max_age: float) -> None:
    check_evidence_identity(response, request, now_ns, max_age)
    if request.stage in (request.GRASP, request.LIFT):
        if not response.contact_confirmed or not response.object_tracked:
            raise RuntimeError('Contact and object tracking are both required')
    elif request.stage == request.RELEASE:
        if not response.release_confirmed or response.contact_confirmed:
            raise RuntimeError('Physical release is not confirmed')
    elif request.stage == getattr(request, 'PLACEMENT', 4):
        if (not request.destination_id or not response.placement_confirmed
                or not response.release_confirmed or response.contact_confirmed):
            raise RuntimeError('Independent destination placement is not confirmed')
    else:
        raise RuntimeError('Unknown manipulation evidence stage')
