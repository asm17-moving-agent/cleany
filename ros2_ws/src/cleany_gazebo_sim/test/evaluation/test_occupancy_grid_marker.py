from math import pi

import pytest

from cleany_gazebo_sim.occupancy_grid_marker import (
    GridGeometry,
    occupied_cell_centers,
)


def test_occupied_cell_centers_filters_and_transforms_cells() -> None:
    geometry = GridGeometry(
        width=2,
        height=2,
        resolution=0.5,
        origin_x=1.0,
        origin_y=2.0,
        origin_yaw=pi / 2.0,
    )

    centers = occupied_cell_centers(
        [-1, 0, 65, 100], geometry, occupied_threshold=65
    )

    assert centers[0] == pytest.approx((0.25, 2.25))
    assert centers[1] == pytest.approx((0.25, 2.75))


@pytest.mark.parametrize(
    ('data', 'geometry', 'threshold'),
    [
        ([0], GridGeometry(0, 1, 1.0, 0.0, 0.0, 0.0), 65),
        ([0], GridGeometry(1, 1, 0.0, 0.0, 0.0, 0.0), 65),
        ([], GridGeometry(1, 1, 1.0, 0.0, 0.0, 0.0), 65),
        ([0], GridGeometry(1, 1, 1.0, 0.0, 0.0, 0.0), 101),
    ],
)
def test_occupied_cell_centers_rejects_invalid_grids(
    data: list[int], geometry: GridGeometry, threshold: int
) -> None:
    with pytest.raises(ValueError):
        occupied_cell_centers(data, geometry, threshold)
