"""公开雨流示例、幅值变换和S-N损伤独立计算检查。"""
import math
import unittest

import numpy as np

from robot_joint.fatigue import rainflow_cycles, sn_damage


class RainflowTests(unittest.TestCase):
    def test_nine_reversal_example(self):
        result = rainflow_cycles([-2, 1, -3, 5, -1, 3, -4, 4, -2])
        expected = [(3, -0.5, 0.5), (4, -1, 0.5), (4, 1, 1), (8, 1, 0.5),
                    (9, 0.5, 0.5), (8, 0, 0.5), (6, 1, 0.5)]
        self.assertEqual([(r["range"], r["mean"], r["count"]) for r in result], expected)

    def test_fourteen_reversal_table(self):
        result = rainflow_cycles([-2, 1, -3, 5, -1, 3, -4, 4, -3, 1, -2, 3, 2, 6])
        expected = [(3, -0.5, 0.5), (4, -1, 0.5), (4, 1, 1), (8, 1, 0.5),
                    (3, -0.5, 1), (1, 2.5, 1), (7, 0.5, 1), (9, 0.5, 0.5), (10, 1, 0.5)]
        self.assertEqual([(r["range"], r["mean"], r["count"]) for r in result], expected)

    def test_platform_and_linear_intermediate_samples(self):
        original = [0, 4, 1, 3, 2, 5, 0]
        oversampled = [0, 0, 2, 4, 4, 2, 1, 2, 3, 2.5, 2, 4, 5, 0, 0]
        self.assertEqual(rainflow_cycles(original), rainflow_cycles(oversampled))

    def test_shift_and_amplitude_scale(self):
        sequence = np.array([0, 4, 1, 3, 2, 5, 0.0])
        original = rainflow_cycles(sequence)
        changed = rainflow_cycles(3 * sequence + 7)
        np.testing.assert_allclose([r["range"] for r in changed], [3 * r["range"] for r in original])
        np.testing.assert_allclose([r["mean"] for r in changed], [3 * r["mean"] + 7 for r in original])
        self.assertEqual([r["count"] for r in changed], [r["count"] for r in original])

    def test_empty_single_constant_and_monotone(self):
        for sequence in ([], [1], [2, 2, 2]):
            self.assertEqual(rainflow_cycles(sequence), [])
        self.assertEqual(rainflow_cycles([0, 1, 2]), [{"range": 2, "mean": 1, "count": 0.5}])

    def test_invalid_input(self):
        for sequence in ([0, math.nan], [0, math.inf], [[0, 1]], [True, False], ["0", "1"]):
            with self.assertRaises(ValueError):
                rainflow_cycles(sequence)


class DamageTests(unittest.TestCase):
    @staticmethod
    def damage(cycles, **kwargs):
        parameters = dict(reference_stress_pa=1e8, reference_cycles=1e6, slope=5)
        parameters.update(kwargs)
        return sn_damage(cycles, **parameters)

    def test_reference_amplitude(self):
        result = self.damage([{"range": 2e8, "mean": 0, "count": 100}])
        self.assertAlmostEqual(result["damage_per_duty"], 100 / 1e6)
        self.assertAlmostEqual(result["repeats_to_miner_one"], 1e4)

    def test_half_cycles_and_miner_sum(self):
        result = self.damage(rainflow_cycles([-1e8, 1e8, -1e8]))
        self.assertAlmostEqual(result["damage_per_duty"], 1 / 1e6)

    def test_goodman_tension(self):
        result = self.damage([{"range": 2e8, "mean": 1e8, "count": 100}], ultimate_strength_pa=5e8)
        self.assertAlmostEqual(result["damage_per_duty"], 100 / 1e6 * 1.25 ** 5)

    def test_compression_has_no_goodman_benefit(self):
        result = self.damage([{"range": 2e8, "mean": -1e8, "count": 100}], ultimate_strength_pa=5e8)
        self.assertAlmostEqual(result["damage_per_duty"], 100 / 1e6)

    def test_zero_amplitude_and_empty_damage(self):
        for cycles in ([], [{"range": 0, "mean": 1e8, "count": 100}]):
            result = self.damage(cycles)
            self.assertEqual(result["damage_per_duty"], 0)
            self.assertIsNone(result["repeats_to_miner_one"])

    def test_invalid_parameters(self):
        cycle = [{"range": 2e8, "mean": 0, "count": 1}]
        for kwargs in ({"reference_stress_pa": 0}, {"reference_cycles": -1}, {"slope": math.inf}):
            with self.assertRaises(ValueError):
                self.damage(cycle, **kwargs)
        for bad in ([{"range": -1, "mean": 0, "count": 1}], [{"range": 1, "mean": 0, "count": -1}], [{}]):
            with self.assertRaises(ValueError):
                self.damage(bad)
        with self.assertRaises(ValueError):
            self.damage([{"range": 2e8, "mean": 5e8, "count": 1}], ultimate_strength_pa=5e8)


if __name__ == "__main__":
    unittest.main()
