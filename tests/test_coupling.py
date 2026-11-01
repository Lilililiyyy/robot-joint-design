"""以可解析齿面规律独立核验LTCA—NGW系统协调与最终切线。

模拟齿面仅替代昂贵的单副计算，系统载荷分配和外层迭代均实际运行。
相位刚度数组对应的周期平均位移采用柔度均值，不复用被测截距公式。
"""
import copy
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from robot_joint.gears import CouplingConvergenceError, coupled_planet_contacts
from robot_joint.planetary import PlanetaryStage, solve_load_sharing, stage_geometry


class CouplingTests(unittest.TestCase):
    def setUp(self):
        self.stage = PlanetaryStage(18, 27, 72, 3, 0.0015, 0.012)
        self.cos_alpha = math.cos(self.stage.pressure_angle_rad)
        self.sharing = {
            "pin_stiffness_n_per_m": 5e8,
            "sun_radial_stiffness_n_per_m": [[3e8, 0.2e8], [0.2e8, 4e8]],
            "initial_mesh_stiffness_n_per_m": 1e8,
            "positive_path_gaps_m": [0, 1e-6, 2e-6],
            "negative_path_gaps_m": [2e-6, 1e-6, 0],
        }
        modifications = {"sun_planet": {}, "planet_ring": {}}
        self.mesh = {
            "modifications": {"positive": copy.deepcopy(modifications),
                              "negative": copy.deepcopy(modifications)},
            "coupling": {"maximum_iterations": 90, "force_relative_tolerance": 1e-9,
                         "path_tolerance_m": 1e-12, "relaxation": 0.7},
        }
        self.phase_stiffness = {"sun_planet": np.array([2e8, 2e8]),
                                "planet_ring": np.array([4e8, 4e8])}
        self.normal_gaps = {"sun_planet": 2e-6, "planet_ring": 1e-6}

    def mock_mesh(self, pair, force_t, options, member_mods, misalignment):
        key = "planet_ring" if pair.wheel.internal else "sun_planet"
        stiffness = self.phase_stiffness[key]
        # 按逐相位线性位移先求周期平均，独立于被测程序如何合并刚度。
        force_normal = float(force_t) / self.cos_alpha
        displacements = force_normal / stiffness + self.normal_gaps[key]
        return SimpleNamespace(
            summary={"mean_normal_approach_m": float(np.mean(displacements)),
                     "max_pressure_pa": 0.0},
            mesh_stiffness_n_per_m=stiffness.copy(),
        )

    def run_coupling(self, torque=30, *, sharing=None, mesh=None):
        with patch("robot_joint.gears._mesh_analysis", side_effect=self.mock_mesh):
            return coupled_planet_contacts(self.stage, torque,
                                           self.sharing if sharing is None else sharing,
                                           self.mesh if mesh is None else mesh)

    def closed_form_reference(self, torque, sharing=None):
        """真实固定刚度/间隙可直接解一次，不依赖外层迭代。"""
        inputs = self.sharing if sharing is None else sharing
        sign = "positive" if torque >= 0 else "negative"
        effective_gap = np.asarray(inputs[sign + "_path_gaps_m"], dtype=float)
        effective_gap = effective_gap + sum(self.normal_gaps.values()) / self.cos_alpha
        return solve_load_sharing(
            self.stage, torque,
            sun_mesh_stiffness_n_per_m=1 / np.mean(1 / self.phase_stiffness["sun_planet"]),
            ring_mesh_stiffness_n_per_m=1 / np.mean(1 / self.phase_stiffness["planet_ring"]),
            pin_stiffness_n_per_m=inputs["pin_stiffness_n_per_m"],
            sun_radial_stiffness_n_per_m=inputs["sun_radial_stiffness_n_per_m"],
            path_gaps_m=effective_gap,
            extra_compliance_m_per_n=inputs.get("extra_compliance_m_per_n"),
        )

    def assert_final_state(self, result, expected):
        self.assertTrue(result["coupling"]["converged"])
        np.testing.assert_allclose(result["tangential_forces_n"], expected["tangential_forces_n"],
                                   rtol=2e-8, atol=2e-8)
        np.testing.assert_allclose(result["compliance_m_per_n"], expected["compliance_m_per_n"],
                                   rtol=2e-12, atol=2e-22)
        np.testing.assert_allclose(result["effective_path_gaps_m"], expected["effective_path_gaps_m"],
                                   rtol=1e-10, atol=1e-15)
        self.assertAlmostEqual(result["output_torsional_stiffness_n_m_per_rad"]
                               / expected["output_torsional_stiffness_n_m_per_rad"], 1, places=10)
        lever = 2 * stage_geometry(self.stage)["carrier_radius_m"]
        self.assertAlmostEqual(result["output_torsional_stiffness_n_m_per_rad"],
                               lever**2 * result["path_tangent_stiffness_n_per_m"], places=8)
        # 最终返回的柔度、截距、力和接近量须属于同一个已协调状态。
        path = (result["compliance_m_per_n"] @ result["tangential_forces_n"]
                + result["effective_path_gaps_m"])
        active = np.asarray(result["active_mask"], dtype=bool)
        np.testing.assert_allclose(path[active], result["path_approach_m"], atol=2e-12, rtol=0)
        self.assertTrue(np.all(path[~active] >= result["path_approach_m"] - 2e-12))

    def test_zero_extremely_low_and_normal_load(self):
        for torque in (0.0, 1e-12, 30.0):
            with self.subTest(torque=torque):
                result, contacts = self.run_coupling(torque)
                self.assert_final_state(result, self.closed_form_reference(torque))
                self.assertEqual(len(contacts["sun_planet"]), 3)
                self.assertEqual(len(contacts["planet_ring"]), 3)
                self.assertAlmostEqual(result["carrier_output_torque_n_m"], torque, places=11)
        zero, _ = self.run_coupling(0.0)
        self.assertEqual(zero["output_elastic_twist_rad"], 0)
        self.assertAlmostEqual(zero["path_approach_m"], sum(self.normal_gaps.values()) / self.cos_alpha)

    def test_initial_stiffness_does_not_change_final_physical_state(self):
        results = []
        for initial in (1e5, 1e8, 1e11):
            sharing = dict(self.sharing, initial_mesh_stiffness_n_per_m=initial)
            result, _ = self.run_coupling(sharing=sharing)
            self.assert_final_state(result, self.closed_form_reference(30, sharing))
            results.append(result)
        for result in results[1:]:
            np.testing.assert_allclose(result["tangential_forces_n"], results[0]["tangential_forces_n"],
                                       rtol=3e-8, atol=3e-8)
            self.assertAlmostEqual(result["output_torsional_stiffness_n_m_per_rad"],
                                   results[0]["output_torsional_stiffness_n_m_per_rad"], places=7)

    def test_final_compliance_uses_real_mesh_tangent_and_preserves_extra_structure(self):
        extra = np.full((3, 3), 2e-9) + np.diag([1e-9, 2e-9, 3e-9])
        sharing = dict(self.sharing, initial_mesh_stiffness_n_per_m=8e5,
                       extra_compliance_m_per_n=extra)
        result, _ = self.run_coupling(sharing=sharing)
        self.assert_final_state(result, self.closed_form_reference(30, sharing))

    def test_phase_stiffness_is_combined_by_mean_compliance(self):
        self.phase_stiffness = {"sun_planet": np.array([2e8, 4e8]),
                                "planet_ring": np.array([1e8, 3e8])}
        result, _ = self.run_coupling()
        expected = self.closed_form_reference(30)
        self.assert_final_state(result, expected)
        arithmetic = solve_load_sharing(
            self.stage, 30,
            sun_mesh_stiffness_n_per_m=np.mean(self.phase_stiffness["sun_planet"]),
            ring_mesh_stiffness_n_per_m=np.mean(self.phase_stiffness["planet_ring"]),
            pin_stiffness_n_per_m=self.sharing["pin_stiffness_n_per_m"],
            sun_radial_stiffness_n_per_m=self.sharing["sun_radial_stiffness_n_per_m"],
        )
        self.assertFalse(np.allclose(result["compliance_m_per_n"], arithmetic["compliance_m_per_n"],
                                    rtol=1e-3, atol=0))

    def test_positive_gap_opens_then_reenters_with_load(self):
        sharing = dict(self.sharing, positive_path_gaps_m=[0, 3e-5, 3e-5])
        low, _ = self.run_coupling(0.1, sharing=sharing)
        high, _ = self.run_coupling(1000.0, sharing=sharing)
        self.assert_final_state(low, self.closed_form_reference(0.1, sharing))
        self.assert_final_state(high, self.closed_form_reference(1000.0, sharing))
        self.assertEqual(np.count_nonzero(low["active_mask"]), 1)
        self.assertEqual(np.count_nonzero(high["active_mask"]), 3)
        self.assertAlmostEqual(low["load_share_factor"], 3)

    def test_reverse_uses_negative_working_face_gap(self):
        result, _ = self.run_coupling(-30)
        self.assert_final_state(result, self.closed_form_reference(-30))
        self.assertLess(result["carrier_output_torque_n_m"], 0)
        self.assertLess(result["output_elastic_twist_rad"], 0)

    def test_nonconvergence_is_explicit(self):
        sharing = dict(self.sharing, initial_mesh_stiffness_n_per_m=1e4)
        mesh = copy.deepcopy(self.mesh)
        mesh["coupling"].update(maximum_iterations=2, relaxation=0.001)
        with self.assertRaisesRegex(CouplingConvergenceError, "2次迭代"):
            self.run_coupling(sharing=sharing, mesh=mesh)

    def test_nonpositive_phase_stiffness_is_rejected(self):
        self.phase_stiffness["sun_planet"] = np.array([0.0, 2e8])
        with np.errstate(divide="ignore", invalid="ignore"):
            with self.assertRaisesRegex(CouplingConvergenceError, "相位刚度"):
                self.run_coupling()


if __name__ == "__main__":
    unittest.main()
