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
        candidate = directory / "examples" / "dfki_pendulum_case.json"
        if candidate.is_file():
            return candidate
    raise FileNotFoundError("未找到NGW集成验证示例")


class JointAnalysisTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original_config = json.loads(_example_path().read_text(encoding="utf-8"))
        # 集成测试只需实际求解一个工况；公开完整任务和64×26离散由案例/收敛脚本复算。
        cls.original_config["duty_segments"] = [cls.original_config["duty_segments"][0]]
        cls.original_config["mesh"]["phases"] = 4
        cls.original_config["mesh"]["strips"] = 3
        cls.original_before = deepcopy(cls.original_config)
        cls.original_result = analyze_joint(cls.original_config)
        cls.small_config = deepcopy(cls.original_config)
        cls.small_config["duty_segments"] = [cls.small_config["duty_segments"][0]]
        cls.small_config["mesh"]["phases"] = 4
        cls.small_config["mesh"]["strips"] = 3
        # 平面摆台本身没有Mx/My；本分支另加给定倾覆矩以实际激活失准接口。
        cls.small_config["duty_segments"][0]["output_wrench"][3:5] = [2.0, -1.0]
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
        config["design_limits"].pop("fatigue_damage_per_input_history", None)
        for key in ("minimum_contact_safety_factor", "minimum_bending_safety_factor"):
            config["design_limits"].pop(key, None)
        result = self._reuse_small_contacts(config)
        self.assertEqual(result.cases[0]["ratings"], {})
        self.assertIn("未配置", result.fatigue["status"])
        self.assertIsNone(result.summary["minimum_contact_safety_factor"])
        self.assertIsNone(result.summary["minimum_bending_safety_factor"])
        self.assertFalse(result.summary["physical_validation"])
        self.assertFalse(result.summary["vendor_system_validation"])

    def test_supplied_fatigue_failure_cannot_pass_design_screen(self):
        config = deepcopy(self.small_config)
        config["design_limits"].pop("fatigue_damage_per_input_history", None)
        # 仅降低S-N参考循环数使给定谱D>1，保持齿面/机电输入不变。
        config["fatigue"]["reference_cycles"] = 0.01
        result = self._reuse_small_contacts(config)
        self.assertGreater(result.summary["fatigue_damage_per_input_history"], 1)
        self.assertFalse(result.summary["checks"]["fatigue_damage_per_input_history"])
        self.assertFalse(result.summary["configured_checks_pass"])
        detail = result.summary["check_details"]["fatigue_damage_per_input_history"]
        self.assertEqual(detail["status"], "failed")
        self.assertEqual(detail["threshold"], 1)
        self.assertEqual(detail["relation"], "<=")
        self.assertIn("给定热点", detail["scope"])

    def test_joint_stiffness_sharing_and_fatigue_limits_have_actual_values(self):
        config = deepcopy(self.small_config)
        config["design_limits"].update({
            "minimum_output_torsional_stiffness_n_m_per_rad": 1e12,
            "maximum_load_share_factor": 1.0,
            "fatigue_damage_per_input_history": 1e-20,
        })
        result = self._reuse_small_contacts(config)
        for name in ("minimum_output_torsional_stiffness_n_m_per_rad", "maximum_load_share_factor",
                     "fatigue_damage_per_input_history"):
            with self.subTest(name=name):
                detail = result.summary["check_details"][name]
                self.assertFalse(result.summary["checks"][name])
                self.assertEqual(detail["status"], "failed")
                self.assertEqual(detail["actual"], result.summary[name])
                self.assertEqual(detail["threshold"], config["design_limits"][name])

    def test_missing_fatigue_and_unconfigured_constraints_remain_unevaluated(self):
        config = deepcopy(self.small_config)
        config.pop("fatigue")
        config["design_limits"].pop("fatigue_damage_per_input_history", None)
        config["design_limits"].pop("minimum_output_torsional_stiffness_n_m_per_rad", None)
        result = self._reuse_small_contacts(config)
        detail = result.summary["check_details"]
        self.assertEqual(detail["fatigue_damage_per_input_history"]["status"], "not_evaluated")
        self.assertNotIn("fatigue_damage_per_input_history", result.summary["checks"])
        self.assertEqual(detail["minimum_output_torsional_stiffness_n_m_per_rad"]["status"], "not_configured")
        self.assertIsNone(detail["minimum_output_torsional_stiffness_n_m_per_rad"]["threshold"])
        self.assertIn("未评估", result.summary["assessment_coverage"]["planet_pin_strength"])
        config["design_limits"]["fatigue_damage_per_input_history"] = 0.5
        with self.assertRaisesRegex(ValueError, "没有已计算"):
            self._reuse_small_contacts(config)

    def test_reported_motor_check_actuals_obey_the_model_units(self):
        result = self.small_result
        detail = result.summary["check_details"]
        self.assertEqual(detail["motor_speed"]["actual"],
                         max(abs(s["motor_speed_rad_s"]) for s in result.motor["segments"]))
        self.assertEqual(detail["motor_speed"]["threshold"], self.small_config["motor"]["maximum_speed_rad_s"])
        self.assertEqual(detail["brake_static_screen"]["actual"], result.motor["brake_available_torque_n_m"])
        self.assertEqual(detail["brake_static_screen"]["threshold"], result.motor["brake_required_torque_n_m"])
        for name, passed in result.summary["checks"].items():
            self.assertEqual(detail[name]["status"], "passed" if passed else "failed")

    def test_stiffness_reference_offset_reaches_structure_and_mesh_alignment(self):
        config = deepcopy(self.small_config)
        config["support"]["reference_offset_m"] = 0.05
        result = analyze_joint(config)
        case = result.cases[0]
        wrench = np.asarray(config["duty_segments"][0]["output_wrench"])
        expected_reference = wrench.copy()
        expected_reference[3:] += np.cross([0, 0, 0.05], wrench[:3])
        np.testing.assert_allclose(case["support"]["reference_wrench"], expected_reference, atol=1e-14)
        rotation = np.array(case["support"]["generalized_displacement"])
        self.assertGreater(np.linalg.norm(rotation-self.small_result.cases[0]["support"]["generalized_displacement"]), 1e-9)
        for name, matrix in config["mesh"]["alignment_maps"].items():
            np.testing.assert_allclose(case["sharing"]["coupling"]["misalignment_m"][name],
                                       np.asarray(matrix) @ rotation, atol=1e-15)


if __name__ == "__main__":
    unittest.main()
