"""NGW降阶模型的独立平衡、虚功、脱开及运动学核验。"""
import math
import unittest

import numpy as np

from robot_joint.planetary import PlanetaryStage, solve_load_sharing, speed_map, stage_geometry


class ValidPlanetaryTests(unittest.TestCase):
    def setUp(self):
        self.stage = PlanetaryStage(18, 27, 72, 3, 0.001, 0.012)
        self.stiffness = {
            "sun_mesh_stiffness_n_per_m": 1e8,
            "ring_mesh_stiffness_n_per_m": 2e8,
            "pin_stiffness_n_per_m": 4e8,
            "sun_radial_stiffness_n_per_m": 3e8,
        }

    def solve(self, torque=30.0, **kwargs):
        return solve_load_sharing(self.stage, torque, **(self.stiffness | kwargs))

    def test_geometry_and_signed_speeds(self):
        geometry = stage_geometry(self.stage)
        self.assertAlmostEqual(geometry["ratio"], 5.0)
        self.assertAlmostEqual(geometry["carrier_radius_m"], 0.0225)
        self.assertEqual(geometry["assembly_index"], 30)
        self.assertGreater(geometry["adjacent_tip_clearance_m"], 0)
        for carrier in (4.0, -4.0, 0.0):
            result = speed_map(self.stage, carrier)
            self.assertAlmostEqual(result["sun_rad_s"], 5 * carrier)
            self.assertAlmostEqual(result["ring_rad_s"], 0)
            relative = result["planet_rad_s"] - carrier
            self.assertAlmostEqual(self.stage.sun_teeth * (result["sun_rad_s"] - carrier)
                                   + self.stage.planet_teeth * relative, 0)
            self.assertAlmostEqual(self.stage.ring_teeth * (-carrier)
                                   - self.stage.planet_teeth * relative, 0)

    def test_equal_sharing_against_closed_form(self):
        result = self.solve()
        lever = 0.045
        expected_force = 30 / lever / 3
        compliance = (1 / 1e8 + 1 / 2e8) / math.cos(self.stage.pressure_angle_rad)**2 + 4 / 4e8
        np.testing.assert_allclose(result["tangential_forces_n"], expected_force, rtol=1e-10)
        np.testing.assert_allclose(result["sun_translation_m"], [0, 0], atol=1e-20)
        self.assertAlmostEqual(result["load_share_factor"], 1.0)
        self.assertAlmostEqual(result["path_approach_m"], expected_force * compliance)
        self.assertAlmostEqual(result["output_torsional_stiffness_n_m_per_rad"], lever**2 * 3 / compliance)
        self.assertAlmostEqual(result["output_elastic_twist_rad"], 30 / result["output_torsional_stiffness_n_m_per_rad"])

    def test_virtual_work_and_energy(self):
        result = self.solve(47.0, path_gaps_m=[0, 1e-6, -0.5e-6])
        force = result["tangential_forces_n"]
        gaps = np.array([0, 1e-6, -0.5e-6])
        compliance = result["compliance_m_per_n"]
        self.assertAlmostEqual(float(force @ (compliance @ force + gaps)),
                               abs(result["carrier_output_torque_n_m"]) * abs(result["output_elastic_twist_rad"]), places=11)
        self.assertLess(abs(result["diagnostics"]["output_torque_balance_n_m"]), 1e-10)
        self.assertLess(abs(result["diagnostics"]["external_torque_balance_n_m"]), 1e-10)
        # 不跨越接触切换时，有限差分刚度应等于凝聚解析切线。
        increment = 1e-3
        higher = self.solve(47.0 + increment, path_gaps_m=gaps)
        observed = increment / (higher["output_elastic_twist_rad"] - result["output_elastic_twist_rad"])
        self.assertAlmostEqual(observed / result["output_torsional_stiffness_n_m_per_rad"], 1.0, places=7)

    def test_unilateral_path_can_open_and_reenter(self):
        low = self.solve(0.1, path_gaps_m=[0, 3e-5, 3e-5])
        self.assertEqual(np.count_nonzero(low["active_mask"]), 1)
        np.testing.assert_allclose(low["tangential_forces_n"][1:], 0, atol=1e-12)
        self.assertAlmostEqual(low["load_share_factor"], 3.0)
        self.assertTrue(np.all(low["path_residual_gaps_m"][1:] > 0))
        high = self.solve(1000, path_gaps_m=[0, 3e-5, 3e-5])
        self.assertEqual(np.count_nonzero(high["active_mask"]), 3)

    def test_reverse_preserves_symmetric_force_and_changes_torque_sign(self):
        forward, reverse = self.solve(30), self.solve(-30)
        np.testing.assert_allclose(forward["tangential_forces_n"], reverse["tangential_forces_n"], rtol=1e-10)
        np.testing.assert_allclose(forward["carrier_pin_forces_xy_n"], -reverse["carrier_pin_forces_xy_n"], atol=1e-10)
        self.assertAlmostEqual(forward["output_elastic_twist_rad"], -reverse["output_elastic_twist_rad"])
        for value in (forward, reverse):
            self.assertAlmostEqual(value["sun_input_torque_n_m"] * 5, value["carrier_output_torque_n_m"])
            self.assertAlmostEqual(value["ring_reaction_torque_n_m"] + value["sun_input_torque_n_m"],
                                   value["carrier_output_torque_n_m"])

    def test_shared_sun_force_matches_support_reaction(self):
        stiffness = np.array([[2e8, 0.4e8], [0.4e8, 3e8]])
        result = self.solve(-12.0, sun_radial_stiffness_n_per_m=stiffness,
                            path_gaps_m=[0, 2e-6, -1e-6])
        np.testing.assert_allclose(stiffness @ result["sun_translation_m"], result["sun_force_xy_n"], rtol=1e-12)
        np.testing.assert_allclose(result["compliance_m_per_n"], result["compliance_m_per_n"].T, atol=1e-22)
        self.assertGreater(np.linalg.eigvalsh(result["compliance_m_per_n"])[0], 0)

    def test_power_and_external_vector_equilibrium(self):
        for torque in (35.0, -35.0):
            result = self.solve(torque, path_gaps_m=[0, 1e-5, -1e-6])
            speeds = speed_map(self.stage, 4.0)
            self.assertAlmostEqual(result["sun_input_torque_n_m"] * speeds["sun_rad_s"],
                                   result["carrier_output_torque_n_m"] * speeds["carrier_rad_s"], places=11)
            np.testing.assert_allclose(result["sun_force_xy_n"] + result["ring_force_xy_n"]
                                       + np.sum(result["carrier_pin_forces_xy_n"], axis=0),
                                       np.zeros(2), atol=2e-12)

    def test_extra_compliance_reduces_output_stiffness(self):
        base = self.solve()
        compliant = self.solve(extra_compliance_m_per_n=np.eye(3) * 1e-8)
        self.assertLess(compliant["output_torsional_stiffness_n_m_per_rad"], base["output_torsional_stiffness_n_m_per_rad"])
        np.testing.assert_allclose(compliant["tangential_forces_n"], base["tangential_forces_n"], rtol=1e-10)

    def test_soft_sun_support_relaxes_three_planet_error_imbalance(self):
        fixed = self.solve(sun_radial_stiffness_n_per_m=1e12, path_gaps_m=[0, 1e-5, 2e-5])
        floating = self.solve(sun_radial_stiffness_n_per_m=1e4, path_gaps_m=[0, 1e-5, 2e-5])
        self.assertLess(floating["load_share_factor"], fixed["load_share_factor"])
        self.assertLess(floating["load_share_factor"], 1.001)
        self.assertGreater(np.linalg.norm(floating["sun_translation_m"]),
                           np.linalg.norm(fixed["sun_translation_m"]))

    def test_zero_load_convention_and_right_tangent(self):
        result = self.solve(0, path_gaps_m=[2e-6, 3e-6, 5e-6])
        np.testing.assert_array_equal(result["tangential_forces_n"], np.zeros(3))
        self.assertEqual(result["output_elastic_twist_rad"], 0.0)
        self.assertEqual(result["path_approach_m"], 2e-6)
        self.assertIsNone(result["load_share_factor"])
        self.assertGreater(result["output_torsional_stiffness_n_m_per_rad"], 0)

    def test_geometric_rejections(self):
        with self.assertRaisesRegex(ValueError, "均布行星装配"):
            PlanetaryStage(20, 30, 80, 3, 0.001, 0.012)
        with self.assertRaisesRegex(ValueError, "相邻行星"):
            PlanetaryStage(20, 30, 80, 5, 0.001, 0.012)
        with self.assertRaisesRegex(ValueError, "ring_teeth"):
            PlanetaryStage(18, 27, 73, 3, 0.001, 0.012)
        with self.assertRaises(ValueError):
            PlanetaryStage(True, 27, 72, 3, 0.001, 0.012)

    def test_standard_rack_undercut_risk_is_explicit_without_rejecting_layout(self):
        original = stage_geometry(self.stage)["undercut_screen"]
        self.assertAlmostEqual(original["minimum_teeth_continuous"], 2/math.sin(math.radians(20))**2)
        self.assertTrue(original["sun_no_undercut_screen"])
        self.assertTrue(original["planet_no_undercut_screen"])
        risky = stage_geometry(PlanetaryStage(12, 24, 60, 3, 0.001, 0.012))["undercut_screen"]
        self.assertFalse(risky["sun_no_undercut_screen"])
        self.assertTrue(risky["planet_no_undercut_screen"])
        self.assertFalse(risky["automatic_design_rejection"])
        self.assertTrue(any("内齿轮" in item for item in risky["unassessed"]))

    def test_invalid_compliance_stiffness_and_load(self):
        cases = [
            {"sun_mesh_stiffness_n_per_m": [1e8, 0, 1e8]},
            {"ring_mesh_stiffness_n_per_m": True},
            {"pin_stiffness_n_per_m": [1e8, 1e8]},
            {"sun_radial_stiffness_n_per_m": [[1e8, 1e8], [0, 1e8]]},
            {"sun_radial_stiffness_n_per_m": [[1e8, 0], [0, -1e8]]},
            {"extra_compliance_m_per_n": -np.eye(3) * 1e-8},
            {"path_gaps_m": [0, float("nan"), 0]},
        ]
        for values in cases:
            with self.subTest(values=values), self.assertRaises(ValueError):
                self.solve(**values)
        for torque in (True, float("inf"), "30"):
            with self.assertRaises(ValueError):
                self.solve(torque)


if __name__ == "__main__":
    unittest.main()
