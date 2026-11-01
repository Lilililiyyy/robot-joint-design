"""根入口的真实LTCA、系统串联、输入契约与已配置约束集成验证。"""
from copy import deepcopy
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from robot_joint.analysis import analyze_joint


def _example_path():
    """按示例标记向上寻找独立项目根，不依赖启动目录。"""
    for directory in Path(__file__).resolve().parents:
        candidate = directory / "examples" / "ngw_18_27_72.json"
        if candidate.is_file():
            return candidate
    raise FileNotFoundError("未找到NGW集成验证示例")


class JointAnalysisTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original_config = json.loads(_example_path().read_text(encoding="utf-8"))
        cls.original_before = deepcopy(cls.original_config)
        cls.original_result = analyze_joint(cls.original_config)
        cls.small_config = deepcopy(cls.original_config)
        cls.small_config["duty_segments"] = [cls.small_config["duty_segments"][0]]
        cls.small_config["mesh"]["phases"] = 4
        cls.small_config["mesh"]["strips"] = 5
        # 独立给定非对角SPD支承，用于核验六分量位移到齿宽失准的接入。
        stiffness = np.asarray(cls.small_config["support"]["stiffness_matrix"], dtype=float)
        stiffness[3, 4] = stiffness[4, 3] = 0.2*math.sqrt(stiffness[3, 3]*stiffness[4, 4])
        cls.small_config["support"]["stiffness_matrix"] = stiffness.tolist()
        cls.small_before = deepcopy(cls.small_config)
        cls.small_result = analyze_joint(cls.small_config)

    def _reuse_small_contacts(self, config):
        """复用已验证的真实接触结果，只隔离本测试未改变的昂贵计算。"""
        case = self.small_result.cases[0]
        output = deepcopy((case["sharing"], case["contacts"]))
        with patch("robot_joint.analysis.coupled_planet_contacts", return_value=output):
            return analyze_joint(config)

    def test_inputs_remain_unchanged_and_raw_contact_arrays_are_retained(self):
        self.assertEqual(self.original_config, self.original_before)
        self.assertEqual(self.small_config, self.small_before)
        self.assertEqual(self.original_result.inputs, self.original_before)
        self.assertEqual(len(self.original_result.cases), len(self.original_config["duty_segments"]))
        for case in self.original_result.cases:
            self.assertEqual(set(case["contacts"]), {"sun_planet", "planet_ring"})
            for group in case["contacts"].values():
                self.assertEqual(len(group), self.original_config["stage"]["planet_count"])
                for contact in group:
                    self.assertEqual(len(contact.rotations), self.original_config["mesh"]["phases"])
                    self.assertGreater(sum(rotation.solution.forces_n.size
                                           for rotation in contact.rotations), 0)
                    self.assertTrue(contact.summary["all_converged"])

    def test_force_balance_projection_and_series_stiffness(self):
        result = self.small_result
        case = result.cases[0]
        sharing = case["sharing"]
        torque = self.small_config["duty_segments"][0]["output_torque_n_m"]
        lever = 2*result.geometry["carrier_radius_m"]
        forces = np.asarray(sharing["tangential_forces_n"])
        self.assertAlmostEqual(float(forces.sum())*lever, abs(torque), places=9)
        np.testing.assert_allclose(sharing["normal_forces_n"],
                                   forces/math.cos(self.small_config["stage"]["pressure_angle_rad"]),
                                   rtol=1e-12)
        np.testing.assert_allclose(sharing["diagnostics"]["external_force_balance_xy_n"],
                                   np.zeros(2), atol=1e-9)
        self.assertAlmostEqual(sharing["output_torsional_stiffness_n_m_per_rad"],
                               lever**2*sharing["path_tangent_stiffness_n_per_m"], places=7)
        support_c = np.linalg.inv(self.small_config["support"]["stiffness_matrix"])
        expected_k = 1/(1/sharing["output_torsional_stiffness_n_m_per_rad"]+support_c[5, 5])
        observed_k = case["precision"]["output_torsional_stiffness_n_m_per_rad"]
        self.assertAlmostEqual(observed_k/expected_k, 1, places=12)
        self.assertLess(observed_k, sharing["output_torsional_stiffness_n_m_per_rad"])
        expected_angle = (sharing["output_elastic_twist_rad"]
                          + case["support"]["generalized_displacement"][5])
        self.assertAlmostEqual(case["precision"]["output_elastic_twist_rad"], expected_angle, places=14)
        self.assertTrue(result.summary["all_couplings_converged"])
        self.assertLess(result.motor["maximum_power_residual_w"], 1e-9)

    def test_support_alignment_maps_reach_actual_ltca_inputs(self):
        case = self.small_result.cases[0]
        displacement = np.asarray(case["support"]["generalized_displacement"])
        nonzero = False
        for name, matrix in self.small_config["mesh"]["alignment_maps"].items():
            expected = np.asarray(matrix) @ displacement
            nonzero = nonzero or bool(np.any(np.abs(expected) > 1e-12))
            np.testing.assert_allclose(case["sharing"]["coupling"]["misalignment_m"][name],
                                       expected, atol=1e-15)
            for contact, value in zip(case["contacts"][name], expected):
                self.assertAlmostEqual(contact.inputs["misalignment_m"], value, places=14)
        self.assertTrue(nonzero)

    def test_inconsistent_output_torque_and_wrench_are_rejected(self):
        config = deepcopy(self.small_config)
        config["duty_segments"][0]["output_wrench"][5] *= -1
        with self.assertRaises(ValueError):
            analyze_joint(config)

    def test_nested_spelling_errors_are_rejected(self):
        variants = []
        config = deepcopy(self.small_config)
        config["load_sharing"]["negative_path_gap_m"] = [0, 0, 0]
        variants.append(("路径间隙拼写", config))
        config = deepcopy(self.small_config)
        config["mesh"]["coupling"]["path_tolerence_m"] = 1e-9
        variants.append(("耦合容差拼写", config))
        config = deepcopy(self.small_config)
        config["mesh"]["modifications"]["negative"]["planet_ring"]["pinnion"] = {}
        variants.append(("工作面角色拼写", config))
        config = deepcopy(self.small_config)
        config["mesh"]["phasez"] = 4
        variants.append(("离散参数拼写", config))
        for label, config in variants:
            with self.subTest(label=label), self.assertRaises(ValueError):
                analyze_joint(config)

    def test_configured_unsafe_material_limits_fail_summary_checks(self):
        config = deepcopy(self.small_config)
        for rating in config["rating"].values():
            for name in ("pinion_limits", "wheel_limits"):
                rating[name]["contact_limit_pa"] = 1.0
                rating[name]["bending_limit_pa"] = 1.0
        result = self._reuse_small_contacts(config)
        self.assertLess(result.summary["minimum_contact_safety_factor"], 1)
        self.assertLess(result.summary["minimum_bending_safety_factor"], 1)
        self.assertFalse(result.summary["configured_checks_pass"])
        self.assertGreaterEqual(sum(not value for value in result.summary["checks"].values()), 2)

    def test_explicit_minimum_safety_constraints_are_applied(self):
        config = deepcopy(self.small_config)
        config["design_limits"]["minimum_contact_safety_factor"] = 1e9
        config["design_limits"]["minimum_bending_safety_factor"] = 1e9
        result = self._reuse_small_contacts(config)
        self.assertFalse(result.summary["configured_checks_pass"])
        self.assertGreaterEqual(sum(not value for value in result.summary["checks"].values()), 2)

    def test_unconfigured_rating_and_fatigue_are_explicit(self):
        config = deepcopy(self.small_config)
        config.pop("rating")
        config.pop("fatigue")
        for key in ("minimum_contact_safety_factor", "minimum_bending_safety_factor"):
            config["design_limits"].pop(key, None)
        result = self._reuse_small_contacts(config)
        self.assertEqual(result.cases[0]["ratings"], {})
        self.assertIn("未配置", result.fatigue["status"])
        self.assertIsNone(result.summary["minimum_contact_safety_factor"])
        self.assertIsNone(result.summary["minimum_bending_safety_factor"])
        self.assertFalse(result.summary["physical_validation"])
        self.assertFalse(result.summary["vendor_system_validation"])


if __name__ == "__main__":
    unittest.main()
