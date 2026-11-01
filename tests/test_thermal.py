"""热网络解析解、半群一致性、段内峰温和能量守恒检查。"""
import math
import unittest

import numpy as np
from scipy.linalg import expm

from robot_joint.thermal import thermal_response, periodic_thermal_response


class ThermalTests(unittest.TestCase):
    @staticmethod
    def solve(segments, **kwargs):
        parameters = dict(capacities_j_per_k=[100, 200], ambient_conductances_w_per_k=[2, 4],
                          coupling_w_per_k=0, ambient_temperature_k=300)
        parameters.update(kwargs)
        return thermal_response(segments, **parameters)

    def test_decoupled_exact_exponential(self):
        result = self.solve([{"duration_s": 50, "motor_loss_w": 20, "gear_loss_w": 40}])
        expected = [300 + 10 * (1 - math.exp(-1)), 300 + 10 * (1 - math.exp(-1))]
        np.testing.assert_allclose(result["final_temperature_k"], expected, atol=1e-12)
        np.testing.assert_allclose(result["maximum_temperature_k"], expected, atol=1e-12)

    def test_split_segment_semigroup(self):
        segment = {"duration_s": 200, "motor_loss_w": 50, "gear_loss_w": 10}
        first = self.solve([segment], coupling_w_per_k=3)
        split = self.solve([{**segment, "duration_s": 80}, {**segment, "duration_s": 120}], coupling_w_per_k=3)
        np.testing.assert_allclose(first["final_temperature_k"], split["final_temperature_k"], atol=1e-12)

    def test_unequal_capacities_match_general_matrix_exponential(self):
        result = self.solve([{"duration_s": 37, "motor_loss_w": 50, "gear_loss_w": 10}],
                            coupling_w_per_k=3, initial_temperature_k=[305, 301])
        capacities = np.diag([100, 200])
        conductance = np.array([[5, -3], [-3, 7.0]])
        equilibrium = np.linalg.solve(conductance, [50, 10])
        expected = 300 + equilibrium + expm(-np.linalg.solve(capacities, conductance) * 37) @ (np.array([5, 1]) - equilibrium)
        np.testing.assert_allclose(result["final_temperature_k"], expected, atol=1e-12)

    def test_steady_initial_condition(self):
        conductance = np.array([[5, -3], [-3, 7.0]])
        steady = np.linalg.solve(conductance, [50, 10]) + 300
        result = self.solve([{"duration_s": 500, "motor_loss_w": 50, "gear_loss_w": 10}],
                            coupling_w_per_k=3, initial_temperature_k=steady)
        np.testing.assert_allclose(result["final_temperature_k"], steady, atol=1e-12)
        np.testing.assert_allclose(result["maximum_temperature_k"], steady, atol=1e-12)

    def test_interior_peak_after_heat_transfer(self):
        result = self.solve([{"duration_s": 40, "motor_loss_w": 0, "gear_loss_w": 0}],
                            capacities_j_per_k=[10, 10], ambient_conductances_w_per_k=[1, 1],
                            coupling_w_per_k=5, initial_temperature_k=[330, 300])
        peak_time = math.log(11)
        peak_gear = 300 + 15 * (math.exp(-0.1 * peak_time) - math.exp(-1.1 * peak_time))
        self.assertAlmostEqual(result["maximum_time_s"][1], peak_time)
        self.assertAlmostEqual(result["maximum_temperature_k"][1], peak_gear)
        self.assertGreater(result["maximum_temperature_k"][1], result["final_temperature_k"][1])

    def test_energy_balance_with_coupling_and_cooling(self):
        result = self.solve([{"duration_s": 80, "motor_loss_w": 30, "gear_loss_w": 10},
                             {"duration_s": 120, "motor_loss_w": 0, "gear_loss_w": 0}], coupling_w_per_k=3)
        self.assertLess(abs(result["diagnostics"]["energy_residual_j"]), 1e-9)
        for segment in result["segments"]:
            self.assertLess(abs(segment["energy_residual_j"]), 1e-9)

    def test_single_ambient_path_still_reaches_equilibrium(self):
        result = self.solve([{"duration_s": 10000, "motor_loss_w": 20, "gear_loss_w": 0}],
                            ambient_conductances_w_per_k=[0, 2], coupling_w_per_k=5)
        np.testing.assert_allclose(result["final_temperature_k"], [314, 310], atol=1e-8)

    def test_sample_arrays_include_boundaries_and_stationary_point(self):
        result = self.solve([{"duration_s": 1, "motor_loss_w": 1, "gear_loss_w": 0},
                             {"duration_s": 2, "motor_loss_w": 0, "gear_loss_w": 1}], coupling_w_per_k=2)
        times = result["time_s"]
        self.assertEqual(times[0], 0)
        self.assertEqual(times[-1], 3)
        self.assertEqual(len(times), len(result["temperature_k"]))
        self.assertTrue(np.all(np.diff(times) > 0))
        self.assertIn(1, times)

    def test_invalid_network_or_parameters(self):
        valid = [{"duration_s": 1, "motor_loss_w": 1, "gear_loss_w": 0}]
        for kwargs in ({"capacities_j_per_k": [0, 1]}, {"coupling_w_per_k": -1},
                       {"ambient_conductances_w_per_k": [0, 0]}, {"ambient_temperature_k": 0},
                       {"initial_temperature_k": [300, math.nan]}, {"capacities_j_per_k": [True, False]}):
            with self.assertRaises(ValueError):
                self.solve(valid, **kwargs)
        for segments in ([], [{"duration_s": 0, "motor_loss_w": 1, "gear_loss_w": 1}],
                         [{"duration_s": 1, "motor_loss_w": -1, "gear_loss_w": 0}], [{}]):
            with self.assertRaises(ValueError):
                self.solve(segments)


class PeriodicThermalTests(unittest.TestCase):
    @staticmethod
    def solve(segments, **kwargs):
        parameters = dict(capacities_j_per_k=[100, 200], ambient_conductances_w_per_k=[2, 4],
                          coupling_w_per_k=3, ambient_temperature_k=300)
        parameters.update(kwargs)
        return periodic_thermal_response(segments, **parameters)

    def test_constant_loss_equals_static_thermal_solution(self):
        result = self.solve([{"duration_s": 30, "motor_loss_w": 50, "gear_loss_w": 10}])
        expected = 300 + np.linalg.solve([[5, -3], [-3, 7]], [50, 10])
        np.testing.assert_allclose(result["periodic_initial_temperature_k"], expected, atol=1e-12)
        np.testing.assert_allclose(result["maximum_temperature_k"], expected, atol=1e-12)
        self.assertTrue(result["periodic_steady"])

    def test_cycle_boundary_and_energy_balance(self):
        result = self.solve([{"duration_s": 8, "motor_loss_w": 100, "gear_loss_w": 20},
                             {"duration_s": 22, "motor_loss_w": 1, "gear_loss_w": 2}])
        self.assertLess(result["periodic_residual_k"], 1e-12)
        self.assertLess(abs(result["diagnostics"]["stored_energy_change_j"]), 1e-9)
        self.assertAlmostEqual(result["diagnostics"]["energy_input_j"],
                               result["diagnostics"]["heat_to_ambient_j"], places=8)
        self.assertEqual(result["period_duration_s"], 30)

    def test_split_constant_segment_keeps_periodic_solution(self):
        first = self.solve([{"duration_s": 30, "motor_loss_w": 50, "gear_loss_w": 10}])
        split = self.solve([{"duration_s": 7, "motor_loss_w": 50, "gear_loss_w": 10},
                            {"duration_s": 23, "motor_loss_w": 50, "gear_loss_w": 10}])
        np.testing.assert_allclose(first["periodic_initial_temperature_k"], split["periodic_initial_temperature_k"], atol=1e-12)

    def test_cyclic_segment_exchange_changes_phase_only(self):
        heating = {"duration_s": 8, "motor_loss_w": 100, "gear_loss_w": 20}
        cooling = {"duration_s": 22, "motor_loss_w": 1, "gear_loss_w": 2}
        result = self.solve([heating, cooling])
        exchanged = self.solve([cooling, heating])
        end_heating_index = result["time_s"].index(8)
        np.testing.assert_allclose(exchanged["periodic_initial_temperature_k"],
                                   result["temperature_k"][end_heating_index], atol=1e-12)
        np.testing.assert_allclose(exchanged["maximum_temperature_k"], result["maximum_temperature_k"], atol=1e-12)

    def test_zero_loss_returns_ambient(self):
        result = self.solve([{"duration_s": 30, "motor_loss_w": 0, "gear_loss_w": 0}])
        np.testing.assert_allclose(result["maximum_temperature_k"], [300, 300], atol=1e-12)
        self.assertEqual(result["periodic_residual_k"], 0)

    def test_short_period_retains_steady_temperature(self):
        result = self.solve([{"duration_s": 1e-9, "motor_loss_w": 20, "gear_loss_w": 40}], coupling_w_per_k=0)
        np.testing.assert_allclose(result["periodic_initial_temperature_k"], [310, 310], atol=1e-12)
        self.assertLess(result["periodic_residual_k"], 1e-12)

    def test_periodic_peak_exceeds_first_cold_cycle(self):
        segments = [{"duration_s": 5, "motor_loss_w": 100, "gear_loss_w": 30},
                    {"duration_s": 25, "motor_loss_w": 0, "gear_loss_w": 0}]
        steady = self.solve(segments)
        first = thermal_response(segments, capacities_j_per_k=[100, 200],
                                 ambient_conductances_w_per_k=[2, 4], coupling_w_per_k=3,
                                 ambient_temperature_k=300)
        self.assertTrue(np.all(np.asarray(steady["maximum_temperature_k"]) > first["maximum_temperature_k"]))


if __name__ == "__main__":
    unittest.main()
