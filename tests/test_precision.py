"""输出角预算的坐标换算和双工作面包络验证。"""
import math
import unittest

from robot_joint.precision import precision_budget, reversal_curve


class PrecisionTests(unittest.TestCase):
    def evaluate(self, **values):
        parameters = {
            "output_elastic_twist_rad": -0.001,
            "output_torsional_stiffness_n_m_per_rad": 10000,
            "output_backlash_rad": 0.0002,
            "encoder_counts_per_revolution": 10000,
            "assembly_error_rad": 0.00005,
        }
        parameters.update(values)
        return precision_budget(**parameters)

    def test_deterministic_budget_adds_absolute_bounds(self):
        result = self.evaluate()
        expected = 0.001+0.0001+math.pi/10000+0.00005
        self.assertAlmostEqual(result["deterministic_worst_case_error_rad"], expected)
        self.assertAlmostEqual(result["backlash_half_width_rad"], 0.0001)

    def test_motor_encoder_quantization_is_referred_to_output(self):
        output = self.evaluate(encoder_on_output=True, ratio=10)
        motor = self.evaluate(encoder_on_output=False, ratio=10)
        self.assertAlmostEqual(output["encoder_output_step_rad"],
                               10*motor["encoder_output_step_rad"])
        self.assertAlmostEqual(motor["encoder_output_half_count_rad"], math.pi/100000)

    def test_budget_does_not_credit_signed_error_cancellation(self):
        positive = self.evaluate(output_elastic_twist_rad=0.001, assembly_error_rad=0.00005)
        negative = self.evaluate(output_elastic_twist_rad=-0.001, assembly_error_rad=-0.00005)
        self.assertEqual(positive["deterministic_worst_case_error_rad"],
                         negative["deterministic_worst_case_error_rad"])

    def test_static_envelope_has_stated_width_and_no_history(self):
        result = reversal_curve([-10, 0, 10, -10], output_backlash_rad=0.002,
                                torsional_stiffness_n_m_per_rad=10000,
                                friction_torque_n_m=2)
        self.assertFalse(result["history_dependent"])
        widths = [a-b for a, b in zip(result["positive_envelope_rad"],
                                     result["negative_envelope_rad"])]
        for width in widths:
            self.assertAlmostEqual(width, 0.0024)
        self.assertEqual(result["positive_envelope_rad"][0],
                         result["positive_envelope_rad"][-1])
        self.assertAlmostEqual(result["positive_envelope_rad"][2], 0.0022)

    def test_zero_backlash_and_friction_reduce_to_linear_elasticity(self):
        result = reversal_curve([-20, 0, 20], output_backlash_rad=0,
                                torsional_stiffness_n_m_per_rad=20000)
        self.assertEqual(result["positive_envelope_rad"], [-0.001, 0, 0.001])
        self.assertEqual(result["negative_envelope_rad"], [-0.001, 0, 0.001])

    def test_invalid_inputs_are_explicit(self):
        for field, value in (("output_backlash_rad", -1),
                             ("output_torsional_stiffness_n_m_per_rad", 0),
                             ("encoder_counts_per_revolution", True),
                             ("encoder_counts_per_revolution", 2.5),
                             ("encoder_counts_per_revolution", 1),
                             ("encoder_on_output", 1), ("ratio", 0),
                             ("assembly_error_rad", math.nan)):
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                self.evaluate(**{field: value})
        with self.assertRaises(ValueError):
            reversal_curve([], output_backlash_rad=0, torsional_stiffness_n_m_per_rad=1)
        with self.assertRaises(ValueError):
            reversal_curve([True], output_backlash_rad=0, torsional_stiffness_n_m_per_rad=1)


if __name__ == "__main__":
    unittest.main()
