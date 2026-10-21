"""Deterministic target sampling and summaries for the native hybrid experiment."""
from __future__ import annotations

from collections import Counter
from dataclasses import asdict, dataclass
import math
import random
from statistics import mean
from typing import Sequence

from cleany_skill_executor.core.physical_grasp_selection import PoseValue


@dataclass(frozen=True)
class TargetCase:
    case_id: str
    arm: str
    category: str
    pose: PoseValue
    opening: float = .4
    witness: tuple[float, ...] | None = None
    witness_valid: bool | None = None
    expected_rejection: bool = False

    def json(self) -> dict:
        return asdict(self)


def joint_samples(limits: Sequence[tuple[float, float]], *, seed: int) -> dict:
    """Generate distinct interiors, joint boundaries and a random candidate pool."""
    rng = random.Random(seed)
    nominal = (0., .65, .8, -.4, 0.)
    interior = []
    for _ in range(32):
        values = tuple(q+rng.uniform(-spread, spread) for q,spread in
            zip(nominal, (.75,.4,.45,.45,1.0), strict=True))
        interior.append(tuple(min(high-.02,max(low+.02,q))
                              for q,(low,high) in zip(values,limits,strict=True)))
    boundary = []
    for index,(low,high) in enumerate(limits):
        for margin in (.005,.04):
            for edge in (low+margin,high-margin):
                values=list(nominal); values[index]=edge
                boundary.append(tuple(values))
    for elbow,wrist in ((.03,-.03),(.03,.03),(-.03,-.03),(-.03,.03)):
        values=list(nominal)
        values[2] = limits[2][1]+elbow if elbow < 0 else limits[2][0]+elbow
        values[4] = limits[4][1]+wrist if wrist < 0 else limits[4][0]+wrist
        boundary.append(tuple(values))
    random_pool=[tuple(rng.uniform(low+.02,high-.02) for low,high in limits)
                 for _ in range(2048)]
    return dict(interior=interior,boundary=boundary,random_pool=random_pool)


def cartesian_offsets() -> list[tuple[float, float, float]]:
    return [(x,y,z) for x in (-.12,-.04,.04,.12)
            for y in (-.12,-.04,.04,.12) for z in (-.08,.08)]


def orientation_offsets() -> list[tuple[int, float]]:
    return [(axis,math.radians(angle)) for axis in range(3)
            for angle in (-150,-90,-60,-30,30,60,90,150)]


def distribution(values: Sequence[float]) -> dict:
    if not values:
        return {'count':0,'mean':None,'p50':None,'p95':None,'maximum':None}
    ordered=sorted(values)
    def quantile(fraction):
        index=(len(ordered)-1)*fraction
        low=math.floor(index); high=math.ceil(index)
        return ordered[low]+(ordered[high]-ordered[low])*(index-low)
    return dict(count=len(values),mean=mean(values),p50=quantile(.5),
                p95=quantile(.95),maximum=max(values))


def summarize(cases: Sequence[dict], rows: Sequence[dict], batches: Sequence[dict]) -> dict:
    lookup={case['case_id']:case for case in cases}
    groups={}
    for category in sorted({case['category'] for case in cases}):
        ids={case['case_id'] for case in cases if case['category']==category}
        selected=[row for row in rows if row['case_id'] in ids]
        known=[row for row in selected if lookup[row['case_id']]['witness_valid'] is True
               and not lookup[row['case_id']]['expected_rejection']]
        plans=[row for row in selected if row.get('plan_attempted')]
        groups[category]=dict(unique_conditions=len(ids),evaluations=len(selected),
            ik_accepted=sum(row['ik_accepted'] for row in selected),
            known_feasible_evaluations=len(known),
            known_feasible_accepted=sum(row['ik_accepted'] for row in known),
            plan_attempts=len(plans),plans_passed=sum(row['plan_passed'] for row in plans),
            plan_seconds=distribution([row['plan_seconds'] for row in plans]),
            failure_reasons=dict(Counter(row['failure'] for row in selected if row.get('failure'))))
    accepted=[row for row in rows if row['ik_accepted']]
    first=[row for row in rows if row['repeat']==0]
    return dict(categories=groups,unique_conditions=len(cases),evaluations=len(rows),
        ik_accepted=len(accepted),unique_conditions_ever_accepted=len({row['case_id'] for row in accepted}),
        first_pass_ik_accepted=sum(row['ik_accepted'] for row in first),
        plan_attempts=sum(row.get('plan_attempted',False) for row in rows),
        plans_passed=sum(row.get('plan_passed',False) for row in rows),
        warm_batch_seconds=distribution([b['seconds'] for b in batches if not b['initializes_solver']]),
        cold_batch_seconds=distribution([b['seconds'] for b in batches if b['initializes_solver']]),
        primary_batches=sum(b['statistics']['primary_batches'] for b in batches),
        fallback_batches=sum(b['statistics']['fallback_batches'] for b in batches),
        infrastructure_errors=sum(bool(row.get('infrastructure_error')) for row in rows),
        expected_rejection_false_accepts=sum(row['ik_accepted'] for row in rows
            if lookup[row['case_id']]['expected_rejection']),
        endpoint_position_error_m=distribution([row['position_error_m'] for row in accepted]))
