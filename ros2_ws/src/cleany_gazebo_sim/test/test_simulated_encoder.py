from math import isclose, pi
from unittest import TestCase

from cleany_gazebo_sim.simulated_encoder import SimulatedQuadratureEncoder


class SimulatedQuadratureEncoderTest(TestCase):
    def encoder(
        self,
        *,
        scales: tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0),
        noise: float = 0.0,
        seed: int = 42,
    ) -> SimulatedQuadratureEncoder:
        return SimulatedQuadratureEncoder(
            ticks_per_revolution=100,
            wheel_scales=scales,
            tick_noise_stddev=noise,
            random_seed=seed,
        )

    def test_one_tick_is_quantized_and_velocity_is_derived(self) -> None:
        encoder = self.encoder()
        encoder.update((0.0, 0.0, 0.0, 0.0), 1.0)
        one_tick_rad = 2.0 * pi / 100.0

        reading = encoder.update(
            (one_tick_rad, one_tick_rad, one_tick_rad, one_tick_rad),
            2.0,
        )

        self.assertEqual(reading.counts, (1, 1, 1, 1))
        self.assertTrue(isclose(reading.positions_rad[0], one_tick_rad))
        self.assertTrue(isclose(reading.velocities_rad_s[0], one_tick_rad))

    def test_fractional_tick_motion_accumulates_between_samples(self) -> None:
        encoder = self.encoder()
        encoder.update((0.0, 0.0, 0.0, 0.0), 0.0)
        quarter_tick_rad = 2.0 * pi / 100.0 / 4.0

        reading = None
        for index in range(1, 5):
            position = quarter_tick_rad * index
            reading = encoder.update(
                (position, position, position, position), float(index)
            )

        assert reading is not None
        self.assertEqual(reading.counts, (1, 1, 1, 1))

    def test_wheel_scale_changes_measured_count(self) -> None:
        encoder = self.encoder(scales=(2.0, 1.0, 1.0, 1.0))
        encoder.update((0.0, 0.0, 0.0, 0.0), 1.0)
        one_tick_rad = 2.0 * pi / 100.0

        reading = encoder.update((one_tick_rad, 0.0, 0.0, 0.0), 2.0)

        self.assertEqual(reading.counts, (2, 0, 0, 0))

    def test_seed_makes_tick_noise_reproducible(self) -> None:
        first = self.encoder(noise=1.0, seed=7)
        second = self.encoder(noise=1.0, seed=7)
        first.update((0.0, 0.0, 0.0, 0.0), 1.0)
        second.update((0.0, 0.0, 0.0, 0.0), 1.0)

        positions = (0.5, 0.5, 0.5, 0.5)
        self.assertEqual(
            first.update(positions, 2.0).counts,
            second.update(positions, 2.0).counts,
        )

    def test_stationary_wheel_does_not_accumulate_random_ticks(self) -> None:
        encoder = self.encoder(noise=10.0)
        initial = encoder.update((0.0, 0.0, 0.0, 0.0), 1.0)

        reading = encoder.update((0.0, 0.0, 0.0, 0.0), 2.0)

        self.assertEqual(reading.counts, initial.counts)
