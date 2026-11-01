"""机电模型的独立解析、功率守恒和边界验证。"""
import math
import unittest

from robot_joint.electromechanical import motor_duty


class MotorDutyTests(unittest.TestCase):
    def setUp(self):
        self.motor = {
            "torque_constant_n_m_per_a": 0.5, "resistance_ohm": 0.2,
            "continuous_torque_n_m": 5.0, "peak_torque_n_m": 10.0,
            "base_speed_rad_s": 100.0, "maximum_speed_rad_s": 300.0,
            "rotor_inertia_kg_m2": 0.01,
        }

    def evaluate(self, segments, **values):
        options = {
            "motor": self.motor, "forward_efficiency": 0.8,
            "backdrive_efficiency": 0.9, "brake_torque_n_m": 5.0,
        }
        options.update(values)
        return motor_duty(segments, 10.0, **options)

    def test_forward_and_reverse_motoring_preserve_power(self):
        segments = [{"duration_s": 2, "output_torque_n_m": sign*20,
                     "output_speed_rad_s": sign*3} for sign in (1, -1)]
        result = self.evaluate(segments)
        for item in result["segments"]:
            self.assertEqual(item["mode"], "motoring")
            self.assertAlmostEqual(abs(item["motor_torque_n_m"]), 2.5)
            self.assertAlmostEqual(item["gearbox_input_power_w"], 75)
            self.assertAlmostEqual(item["output_power_w"], 60)
            self.assertAlmostEqual(item["gear_loss_w"], 15)
            self.assertAlmostEqual(item["power_residual_w"], 0)
        self.assertAlmostEqual(result["rms_current_a"], 5)

    def test_backdrive_returns_less_energy_than_output_supplies(self):
        result = self.evaluate([{"duration_s": 1, "output_torque_n_m": -20,
                                 "output_speed_rad_s": 3}])
        item = result["segments"][0]
        self.assertEqual(item["mode"], "backdriving")
        self.assertAlmostEqual(item["gearbox_input_power_w"], -54)
        self.assertAlmostEqual(item["gear_loss_w"], 6)
        self.assertGreater(item["electrical_power_w"], -54)
        self.assertLess(item["electrical_power_w"], 0)
        self.assertAlmostEqual(item["power_residual_w"], 0)

    def test_zero_speed_holding_retains_copper_loss(self):
        result = self.evaluate([{"duration_s": 4, "output_torque_n_m": 20,
                                 "output_speed_rad_s": 0}])
        item = result["segments"][0]
        self.assertEqual(item["mode"], "holding")
        self.assertAlmostEqual(item["motor_torque_n_m"], 2)
        self.assertAlmostEqual(item["copper_loss_w"], 3.2)
        self.assertEqual(item["gear_loss_w"], 0)
        self.assertAlmostEqual(result["energy"]["electrical_energy_j"], 12.8)

    def test_supplied_acceleration_adds_rotor_inertial_power(self):
        result = self.evaluate([{"duration_s": 1, "output_torque_n_m": 0,
                                 "output_speed_rad_s": 4,
                                 "output_acceleration_rad_s2": 2}])
        item = result["segments"][0]
        self.assertAlmostEqual(item["rotor_inertia_torque_n_m"], 0.2)
        self.assertAlmostEqual(item["rotor_inertia_power_w"], 8)
        self.assertEqual(item["gear_loss_w"], 0)
        self.assertAlmostEqual(item["motor_mechanical_power_w"], 8)
        self.assertAlmostEqual(item["power_residual_w"], 0)

    def test_constant_power_region_and_speed_limit(self):
        result = self.evaluate([{"duration_s": 1, "output_torque_n_m": 24,
                                 "output_speed_rad_s": 20}])
        item = result["segments"][0]
        self.assertEqual(item["continuous_torque_limit_n_m"], 2.5)
        self.assertEqual(item["peak_torque_limit_n_m"], 5)
        self.assertFalse(item["within_continuous_torque_limit"])
        self.assertTrue(item["within_peak_torque_limit"])
        self.assertFalse(result["continuous_duty_screen_pass"])
        too_fast = self.evaluate([{"duration_s": 1, "output_torque_n_m": 0,
                                   "output_speed_rad_s": 31}])
        self.assertFalse(too_fast["within_speed_limit"])

    def test_rms_weighting_uses_duration_not_number_of_segments(self):
        result = self.evaluate([
            {"duration_s": 1, "output_torque_n_m": 20, "output_speed_rad_s": 0},
            {"duration_s": 3, "output_torque_n_m": 0, "output_speed_rad_s": 0},
        ])
        self.assertAlmostEqual(result["rms_current_a"], 2)
        self.assertAlmostEqual(result["rms_motor_torque_n_m"], 1)
        self.assertAlmostEqual(result["normalized_continuous_torque_rms"], 0.2)

    def test_brake_uses_load_ratio_and_factor_without_efficiency_credit(self):
        segments = [{"duration_s": 1, "output_torque_n_m": -40,
                     "output_speed_rad_s": 0}]
        result = self.evaluate(segments)
        self.assertEqual(result["brake_required_torque_n_m"], 6)
        self.assertFalse(result["brake_static_screen_pass"])
        enough = self.evaluate(segments, brake_torque_n_m=6)
        self.assertTrue(enough["brake_static_screen_pass"])

    def test_unity_efficiency_has_no_gear_loss(self):
        for torque in (-20, 20):
            result = self.evaluate([{"duration_s": 1, "output_torque_n_m": torque,
                                     "output_speed_rad_s": 3}],
                                   forward_efficiency=1, backdrive_efficiency=1)
            self.assertEqual(result["segments"][0]["gear_loss_w"], 0)
            self.assertEqual(result["segments"][0]["gearbox_power_residual_w"], 0)

    def test_invalid_parameters_are_rejected(self):
        segment = {"duration_s": 1, "output_torque_n_m": 20, "output_speed_rad_s": 3}
        for value in (0, -0.1, 1.1, math.nan, True):
            with self.subTest(efficiency=value), self.assertRaises(ValueError):
                self.evaluate([segment], forward_efficiency=value)
        for field, value in (("duration_s", 0), ("duration_s", True),
                             ("output_speed_rad_s", math.inf)):
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                self.evaluate([{**segment, field: value}])
        with self.assertRaises(ValueError):
            self.evaluate([])
        with self.assertRaises(ValueError):
            self.evaluate([segment], motor={**self.motor, "maximum_speed_rad_s": 50})
        with self.assertRaises(ValueError):
            self.evaluate([segment], motor={**self.motor, "torque_constant_n_m_per_a": 0})


if __name__ == "__main__":
    unittest.main()
