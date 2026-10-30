"""Choose one classified target from an actual detector snapshot."""
from dataclasses import dataclass
import math

from .target_filter import select_closest_eligible


@dataclass(frozen=True)
class AutomaticTarget:
    snapshot_id: str
    object_id: int
    skill_name: str
    destination_id: str


def select_automatic_target(snapshot, config: dict, *, minimum_confidence: float = .25,
                            denylist: tuple[str, ...] = ('battery', 'knife', 'broken glass'),
                            maximum_smaller_extent_m: float = .11) -> AutomaticTarget:
    if not math.isfinite(minimum_confidence) or not 0 < minimum_confidence <= 1:
        raise ValueError('Minimum confidence must be finite and in (0, 1]')
    if not snapshot.snapshot_id:
        raise ValueError('Detector returned no snapshot identity')
    detections = [item for item in snapshot.detections
                  if item.object_id > 0 and item.sorting_category in ('trash', 'lost_item')
                  and math.isfinite(item.confidence)
                  and item.confidence >= minimum_confidence and item.distance_valid
                  and math.isfinite(item.distance_m) and item.distance_m > 0]
    selection = select_closest_eligible(detections,
        denylist=denylist, maximum_smaller_extent_m=maximum_smaller_extent_m)
    # Screening uses camera depth; target priority uses the robot-frame distance.
    ranked_indices = sorted(selection.eligible_indices, key=lambda index: (
        detections[index].distance_m, detections[index].object_id))
    destinations = config.get('label_destinations', {})
    for index in ranked_indices:
        target = detections[index]
        destination = destinations.get(target.label, destinations.get('*'))
        if destination:
            return AutomaticTarget(snapshot.snapshot_id, int(target.object_id),
                'collect_trash' if target.sorting_category == 'trash' else 'collect_lost_item', destination)
    raise ValueError('No classified, reachable-size target with valid depth and destination policy')
