"""用独立能量解析式与阈值事件核验新写母线初筛。"""
from copy import deepcopy
import math
import unittest

from robot_joint.dc_bus import dc_bus_screen
from robot_joint.electromechanical import motor_duty


class DCBusTests(unittest.TestCase):
    @staticmethod
    def evaluate(segments, **kwargs):
        values = {'capacitance_f': .01, 'source_voltage_v': 20., 'minimum_voltage_v': 15.,
                  'maximum_voltage_v': 30., 'inverter_motoring_efficiency': 1.,
                  'inverter_regenerating_efficiency': 1.}
        values.update(kwargs)
        return dc_bus_screen(segments, **values)

    def test_constant_regeneration_matches_capacitor_energy(self):
        result = self.evaluate([{'duration_s': 2., 'electrical_power_w': -3.}])
        self.assertAlmostEqual(result['maximum_voltage_v'], math.sqrt(20**2+2*6/.01))
        self.assertAlmostEqual(result['regenerated_dc_energy_j'], 6.)
        self.assertAlmostEqual(result['minimum_capacitance_f_for_nominal_start'], 2*6/(30**2-20**2))
        self.assertAlmostEqual(result['additional_peak_energy_buffer_required_j'], 6-2.5)
        self.assertFalse(result['within_maximum_voltage'])
        crossing = next(event for event in result['events'] if event['event']=='maximum_voltage_crossed_up')
        self.assertAlmostEqual(crossing['time_s'], 2.5/3)
        self.assertAlmostEqual(result['total_energy_balance_residual_j'], 0.)

    def test_capacitor_supplies_later_motoring_until_exact_floor_event(self):
        result = self.evaluate([{'duration_s': 2., 'electrical_power_w': -3.},
                                {'duration_s': 2., 'electrical_power_w': 4.}])
        self.assertEqual(result['voltage_v'][-1], 20.)
        self.assertAlmostEqual(result['source_delivered_energy_j'], 2.)
        self.assertAlmostEqual(result['capacitor_reused_for_motoring_energy_j'], 6.)
        event = next(event for event in result['events'] if event['event']=='source_voltage_reached')
        self.assertAlmostEqual(event['time_s'], 3.5)
        crossing = next(event for event in result['events'] if event['event']=='maximum_voltage_crossed_down')
        self.assertAlmostEqual(crossing['time_s'], 2+(6-2.5)/4)
        self.assertLess(result['maximum_segment_energy_residual_j'], 1e-12)

    def test_bidirectional_efficiency_and_inverter_heat_are_consistent(self):
        result = self.evaluate([{'duration_s': 1., 'electrical_power_w': 8.},
                                {'duration_s': 1., 'electrical_power_w': -8.}],
                               inverter_motoring_efficiency=.8,
                               inverter_regenerating_efficiency=.5)
        self.assertAlmostEqual(result['segments'][0]['dc_power_w'], 10.)
        self.assertAlmostEqual(result['segments'][1]['dc_power_w'], -4.)
        self.assertAlmostEqual(result['inverter_loss_energy_j'], 6.)
        self.assertAlmostEqual(result['source_delivered_energy_j'], 10.)
        self.assertAlmostEqual(result['final_excess_capacitor_energy_j'], 4.)
        self.assertAlmostEqual(result['total_energy_balance_residual_j'], 0.)

    def test_absorber_at_floor_cannot_remove_more_than_returned_energy(self):
        result = self.evaluate([{'duration_s': 2., 'electrical_power_w': -3.}],
                               absorption_power_limit_w=10.)
        self.assertEqual(result['voltage_v'][-1], 20.)
        self.assertEqual(result['externally_absorbed_energy_j'], 6.)
        self.assertEqual(result['maximum_external_absorbed_power_w'], 3.)
        self.assertEqual(result['average_external_absorbed_power_w'], 3.)
        self.assertEqual(result['minimum_capacitance_f_for_nominal_start'], 0.)
        self.assertTrue(result['within_maximum_voltage'])

    def test_absorber_can_remove_initial_excess_and_switch_at_floor(self):
        result = self.evaluate([{'duration_s': 2., 'electrical_power_w': -3.}],
                               absorption_power_limit_w=5., initial_voltage_v=math.sqrt(600))
        # 初始U=1J，吸收器的净放电率为2W，0.5s后到源电压；总吸收7J。
        event = next(event for event in result['events'] if event['event']=='source_voltage_reached')
        self.assertAlmostEqual(event['time_s'], .5)
        self.assertAlmostEqual(result['externally_absorbed_energy_j'], 7.)
        self.assertEqual(result['maximum_external_absorbed_power_w'], 5.)
        self.assertAlmostEqual(result['average_external_absorbed_power_w'], 3.5)
        self.assertAlmostEqual(result['total_energy_balance_residual_j'], 0.)

    def test_limited_absorption_reduces_regeneration_without_clamping_upper_limit(self):
        result = self.evaluate([{'duration_s': 2., 'electrical_power_w': -3.}],
                               absorption_power_limit_w=1.)
        self.assertAlmostEqual(result['final_excess_capacitor_energy_j'], 4.)
        self.assertAlmostEqual(result['externally_absorbed_energy_j'], 2.)
        self.assertFalse(result['within_maximum_voltage'])
        self.assertGreater(result['maximum_voltage_v'], 30.)

    def test_mechanical_backdrive_does_not_automatically_mean_dc_regeneration(self):
        parameters = {'torque_constant_n_m_per_a': 1., 'resistance_ohm': 10.,
                      'continuous_torque_n_m': 2., 'peak_torque_n_m': 3.,
                      'base_speed_rad_s': 10., 'maximum_speed_rad_s': 20.,
                      'rotor_inertia_kg_m2': .1}
        motor = motor_duty([{'duration_s': 1., 'output_torque_n_m': -2.,
                            'output_speed_rad_s': 1., 'output_acceleration_rad_s2': -1.}],
                           2., motor=parameters, forward_efficiency=1., backdrive_efficiency=1.,
                           brake_torque_n_m=2.)
        segment = motor['segments'][0]
        self.assertLess(segment['motor_mechanical_power_w'], 0.)
        self.assertGreater(segment['electrical_power_w'], 0.)
        self.assertAlmostEqual(segment['electrical_power_w'],
                               segment['motor_mechanical_power_w']+segment['motor_loss_w'])
        result = self.evaluate(motor['segments'])
        self.assertEqual(result['regenerated_dc_energy_j'], 0.)
        self.assertEqual(result['maximum_voltage_v'], 20.)

    def test_energy_recurrence_is_independent_of_segment_subdivision(self):
        coarse = self.evaluate([{'duration_s': 2., 'electrical_power_w': -3.},
                                {'duration_s': 2., 'electrical_power_w': 4.}])
        fine = self.evaluate([{'duration_s': .25, 'electrical_power_w': p}
                              for p in [-3.]*8+[4.]*8])
        for key in ('maximum_voltage_v', 'source_delivered_energy_j', 'final_excess_capacitor_energy_j',
                    'regenerated_dc_energy_j', 'minimum_capacitance_f_for_nominal_start'):
            self.assertAlmostEqual(coarse[key], fine[key])

    def test_inputs_unchanged_and_invalid_parameters_rejected(self):
        segments = [{'duration_s': 2., 'electrical_power_w': -3.}]
        before = deepcopy(segments)
        self.evaluate(segments)
        self.assertEqual(segments, before)
        for kwargs in ({'capacitance_f': 0}, {'capacitance_f': True},
                       {'inverter_regenerating_efficiency': 1.1},
                       {'absorption_power_limit_w': -1}, {'source_voltage_v': 31},
                       {'initial_voltage_v': 19}, {'minimum_voltage_v': 21},
                       {'maximum_voltage_v': math.nan}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                self.evaluate(segments, **kwargs)
        for bad in ([], [{'duration_s': 0, 'electrical_power_w': 1}],
                    [{'duration_s': 1, 'electrical_power_w': math.inf}],
                    [{'duration_s': 1}], [{'duration_s': True, 'electrical_power_w': 1}]):
            with self.subTest(segments=bad), self.assertRaises(ValueError):
                self.evaluate(bad)


if __name__ == '__main__':
    unittest.main()
