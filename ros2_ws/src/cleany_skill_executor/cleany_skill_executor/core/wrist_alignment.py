"""Analytical wrist-roll candidates ported from bac2a16; FK revalidation required."""
import math


def aligned_wrist_rolls(*, approach: tuple[float, float, float],
                       closing: tuple[float, float, float],
                       desired: tuple[float, float, float], current: float,
                       lower: float, upper: float, sign_invariant: bool) -> tuple[float, ...]:
    """Rotate a collinear-roll/TCP tool; caller must recheck FK and collisions."""
    if not all(math.isfinite(x) for x in (*approach, *closing, *desired, current, lower, upper)) or lower >= upper:
        raise ValueError('wrist alignment needs finite vectors and ordered limits')
    norm = math.sqrt(sum(x*x for x in approach))
    if norm < 1e-9:
        raise ValueError('wrist alignment axis is zero')
    axis = tuple(x/norm for x in approach)

    def projected(vector):
        dot = sum(x*y for x, y in zip(vector, axis, strict=True))
        result = tuple(x-dot*y for x, y in zip(vector, axis, strict=True))
        length = math.sqrt(sum(x*x for x in result))
        return tuple(x/length for x in result) if length > 1e-9 else None

    source, target = projected(closing), projected(desired)
    if source is None or target is None:
        return ()
    cross = (source[1]*target[2]-source[2]*target[1], source[2]*target[0]-source[0]*target[2],
             source[0]*target[1]-source[1]*target[0])
    angle = math.atan2(sum(x*y for x, y in zip(axis, cross, strict=True)),
                       sum(x*y for x, y in zip(source, target, strict=True)))
    candidates = []
    for reversal in ((0., math.pi) if sign_invariant else (0.,)):
        value = current+angle+reversal
        first, last = math.ceil((lower-value)/math.tau), math.floor((upper-value)/math.tau)
        candidates.extend(value+k*math.tau for k in range(first, last+1))
    return tuple(sorted(candidates, key=lambda x: abs(x-current)))
