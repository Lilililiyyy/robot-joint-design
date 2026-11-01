"""支承响应、静力平衡和基本轴承寿命的独立解析检查。"""
import math
import unittest

import numpy as np

from robot_joint.support import support_response, two_bearing_reactions, bearing_duty_life


class SupportTests(unittest.TestCase):
    def test_diagonal_response_and_energy(self):
        stiffness = np.diag([1e8, 2e8, 3e8, 1e5, 2e5, 3e5])
        load = np.array([100, -200, 300, -10, 20, -30.0])
        result = support_response(load, stiffness)
        expected = load / np.diag(stiffness)
        np.testing.assert_allclose(result["generalized_displacement"], expected, atol=1e-18)
        self.assertAlmostEqual(result["strain_energy_j"], np.dot(load, expected) / 2)
        self.assertLess(result["diagnostics"]["relative_scaled_residual"], 1e-14)

    def test_coupled_matrix_reciprocity(self):
        scaling = np.diag([1e4, 2e4, 1e4, 200, 300, 100])
        normalized = np.eye(6)
        normalized[0, 4] = normalized[4, 0] = 0.2
        stiffness = scaling @ normalized @ scaling
        a, b = np.array([100, 0, 0, 0, 0, 0]), np.array([0, 0, 0, 0, 10, 0])
        ua = np.array(support_response(a, stiffness)["generalized_displacement"])
        ub = np.array(support_response(b, stiffness)["generalized_displacement"])
        self.assertAlmostEqual(float(a @ ub), float(b @ ua), places=15)
        np.testing.assert_allclose(stiffness @ ua, a, atol=1e-12)

    def test_unconstrained_and_nonreciprocal_rejected(self):
        invalid = [np.zeros((6, 6)), np.diag([1, 1, 1, 1, 1, -1])]
        nonsymmetric = np.eye(6)
        nonsymmetric[0, 1] = 0.2
        invalid.append(nonsymmetric)
        for stiffness in invalid:
            with self.assertRaises(ValueError):
                support_response([1, 0, 0, 0, 0, 0], stiffness)

    def test_centered_force_reactions(self):
        result = two_bearing_reactions([100, -200, 60, 0, 0, 30], 0.04)
        np.testing.assert_allclose(result["reaction_vectors_n"], [[-50, 100, -30], [-50, 100, -30]])
        self.assertEqual(result["diagnostics"]["axial_torque_carried_by_drivetrain_n_m"], 30)

    def test_overhung_force_and_moment_balance(self):
        load = [100, 200, 60, 2, -3, 17]
        result = two_bearing_reactions(load, 0.04, load_offset_m=0.07, axial_share=1)
        reactions = np.array(result["reaction_vectors_n"])
        total_moment = np.cross([0, 0, 0.07], load[:3]) + load[3:]
        np.testing.assert_allclose(reactions.sum(axis=0), -np.array(load[:3]), atol=1e-12)
        reaction_moment = np.cross([0, 0, -0.02], reactions[0]) + np.cross([0, 0, 0.02], reactions[1])
        np.testing.assert_allclose((reaction_moment + total_moment)[:2], [0, 0], atol=1e-12)
        np.testing.assert_allclose(result["axial_loads_n"], [60, 0])

    def test_pure_bending_pair(self):
        result = two_bearing_reactions([0, 0, 0, 4, -2, 0], 0.1)
        np.testing.assert_allclose(result["reaction_vectors_n"], [[-20, -40, 0], [20, 40, 0]])

    def test_reaction_invalid_parameters(self):
        for kwargs in ({"spacing_m": 0}, {"spacing_m": 1, "axial_share": 1.1},
                       {"spacing_m": True}, {"spacing_m": 1, "load_offset_m": math.nan}):
            with self.assertRaises(ValueError):
                two_bearing_reactions([0] * 6, **kwargs)


class BearingLifeTests(unittest.TestCase):
    @staticmethod
    def evaluate(radial=(1000,), axial=(0,), speed=(20 * math.pi,), duration=(3600,), **kwargs):
        parameters = dict(dynamic_capacity_n=10000, static_capacity_n=4000, axial_factor=0)
        parameters.update(kwargs)
        return bearing_duty_life(radial, axial, speed, duration, **parameters)

    def test_constant_load_matches_l10_hours(self):
        result = self.evaluate()
        self.assertAlmostEqual(result["life_hours"], 1e6 / (60 * 600) * (10000 / 1000) ** 3)
        self.assertAlmostEqual(result["minimum_static_safety_factor"], 4)
        self.assertAlmostEqual(result["equivalent_dynamic_load_n"], 1000)

    def test_doubled_ball_bearing_load_gives_eightfold_damage(self):
        first, doubled = self.evaluate(), self.evaluate(radial=[2000])
        self.assertAlmostEqual(doubled["damage_per_duty"] / first["damage_per_duty"], 8)

    def test_rotations_weight_variable_load(self):
        result = self.evaluate(radial=[1000, 2000], axial=[0, 0], speed=[2 * math.pi, 4 * math.pi], duration=[100, 100])
        expected = ((1000 ** 3 * 100 + 2000 ** 3 * 200) / 300) ** (1 / 3)
        self.assertAlmostEqual(result["equivalent_dynamic_load_n"], expected)
        self.assertAlmostEqual(result["damage_per_duty"], ((0.1 ** 3 * 100) + (0.2 ** 3 * 200)) / 1e6)

    def test_reversal_keeps_rotation_damage(self):
        self.assertAlmostEqual(self.evaluate(speed=[-20 * math.pi])["damage_per_duty"], self.evaluate()["damage_per_duty"])

    def test_stationary_load_has_static_check(self):
        result = self.evaluate(speed=[0])
        self.assertIsNone(result["life_hours"])
        self.assertIsNone(result["equivalent_dynamic_load_n"])
        self.assertEqual(result["damage_per_duty"], 0)
        self.assertEqual(result["minimum_static_safety_factor"], 4)

    def test_combined_load_uses_supplied_catalog_branch(self):
        result = self.evaluate(axial=[200], radial_factor=0.56, axial_factor=1.5)
        self.assertAlmostEqual(result["dynamic_equivalent_load_n"][0], 860)

    def test_roller_exponent(self):
        result = self.evaluate(exponent=10 / 3)
        self.assertAlmostEqual(result["life_hours"], 1e6 / (60 * 600) * 10 ** (10 / 3))

    def test_invalid_duties_rejected(self):
        for kwargs in ({"radial": [-1]}, {"duration": [0]}, {"speed": [math.inf]},
                       {"axial": [0, 0]}, {"dynamic_capacity_n": 0}, {"exponent": True}):
            with self.assertRaises(ValueError):
                self.evaluate(**kwargs)


if __name__ == "__main__":
    unittest.main()
