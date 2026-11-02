"""主流程可选销/轴承与母线校核的真实接入、失败传播和兼容性检查。"""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from robot_joint import JointResult, analyze_joint, export_joint, motor_duty
from examples.check_planet_support import default_assumptions


class OptionalScreenTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = next(p for p in Path(__file__).resolve().parents if (p/'pyproject.toml').is_file())
        base = json.loads((root/'examples/dfki_pendulum_case.json').read_text(encoding='utf-8'))
        motor_segments = [{key: val for key, val in state.items() if key in
            {'duration_s', 'output_torque_n_m', 'output_speed_rad_s', 'output_acceleration_rad_s2'}}
            for state in base['duty_segments']]
        motor = motor_duty(motor_segments, 6., motor=base['motor'], **base['drive'])
        selected = min(range(len(motor['segments'])), key=lambda j: motor['segments'][j]['electrical_power_w'])
        assert motor['segments'][selected]['electrical_power_w'] < 0
        base['duty_segments'] = [base['duty_segments'][selected]]
        base['mesh'].update(phases=4, strips=3)
        base.pop('fatigue', None)
        base.pop('design_limits', None)
        cls.base = base
        cls.actual = analyze_joint(base)
        cls.pin_option = {k: v for k, v in default_assumptions()['candidates'][1].items() if k != 'id'}
        cls.bus_option = {'capacitance_f': .01, 'source_voltage_v': 48., 'minimum_voltage_v': 42.,
                          'maximum_voltage_v': 60., 'inverter_motoring_efficiency': .95,
                          'inverter_regenerating_efficiency': .95, 'absorption_power_limit_w': 100.}

    def evaluate(self, config):
        # 所有测试只改独立后处理参数，复用同一组已真实求解的接触，避免无关LTCA重复。
        source = self.actual.cases[0]
        with patch('robot_joint.analysis.coupled_planet_contacts',
                   return_value=deepcopy((source['sharing'], source['contacts']))):
            return analyze_joint(config)

    def config(self):
        value = deepcopy(self.base)
        value.update(planet_support=deepcopy(self.pin_option), dc_bus=deepcopy(self.bus_option))
        return value

    def test_missing_options_preserve_old_constructor_and_not_evaluated_state(self):
        result = self.actual
        self.assertIsNone(result.planet_support)
        self.assertIsNone(result.dc_bus)
        self.assertEqual(result.summary['check_details']['planet_pin_static']['status'], 'not_evaluated')
        self.assertEqual(result.summary['check_details']['dc_bus_maximum_voltage']['status'], 'not_evaluated')
        old = JointResult(result.inputs, result.geometry, result.cases, result.motor,
                          result.thermal, result.bearings, result.fatigue, result.summary)
        self.assertIsNone(old.planet_support)
        self.assertIsNone(old.dc_bus)

    def test_both_screens_pass_and_voltage_floor_remains_an_assumption(self):
        config = self.config()
        before = deepcopy(config)
        result = self.evaluate(config)
        self.assertEqual(config, before)
        self.assertIsNotNone(result.planet_support)
        self.assertIsNotNone(result.dc_bus)
        self.assertTrue(result.summary['checks']['planet_pin_static'])
        self.assertTrue(result.summary['checks']['dc_bus_maximum_voltage'])
        self.assertTrue(result.summary['configured_checks_pass'])
        self.assertEqual(result.summary['check_details']['dc_bus_minimum_voltage']['status'], 'not_evaluated')
        self.assertNotIn('dc_bus_minimum_voltage', result.summary['checks'])
        self.assertEqual(result.summary['check_details']['minimum_planet_bearing_basic_l10_hours']['status'], 'not_configured')
        self.assertFalse(result.planet_support['summary']['system_stiffness_was_updated'])

    def test_pin_failure_enters_overall_screening(self):
        config = self.config()
        config['planet_support']['pins']['allowable_bending_stress_pa'] = 1.
        result = self.evaluate(config)
        self.assertFalse(result.summary['checks']['planet_pin_static'])
        self.assertFalse(result.summary['configured_checks_pass'])
        self.assertGreater(result.summary['maximum_pin_static_utilization'], 1.)

    def test_regenerative_overvoltage_enters_overall_screening(self):
        config = self.config()
        config['dc_bus'].update(capacitance_f=1e-6, maximum_voltage_v=50., absorption_power_limit_w=0.)
        result = self.evaluate(config)
        self.assertLess(result.motor['segments'][0]['electrical_power_w'], 0)
        self.assertFalse(result.summary['checks']['dc_bus_maximum_voltage'])
        self.assertFalse(result.summary['configured_checks_pass'])
        self.assertGreater(result.summary['maximum_dc_bus_voltage_v'], 50.)

    def test_static_strength_and_explicit_stiffness_consistency_are_separate(self):
        config = self.config()
        config['planet_support']['pins']['diameter_m'] = .004
        result = self.evaluate(config)
        self.assertTrue(result.summary['checks']['planet_pin_static'])
        self.assertEqual(result.summary['check_details']['maximum_pin_stiffness_relative_difference']['status'], 'not_configured')
        config['design_limits'] = {'maximum_pin_stiffness_relative_difference': .1}
        constrained = self.evaluate(config)
        self.assertTrue(constrained.summary['checks']['planet_pin_static'])
        self.assertFalse(constrained.summary['checks']['maximum_pin_stiffness_relative_difference'])
        self.assertFalse(constrained.summary['configured_checks_pass'])
        self.assertEqual(constrained.inputs['load_sharing']['pin_stiffness_n_per_m'], self.base['load_sharing']['pin_stiffness_n_per_m'])

    def test_bearing_life_limit_requires_explicit_input_and_is_enforced(self):
        config = self.config()
        config['design_limits'] = {'minimum_planet_bearing_basic_l10_hours': 1e30}
        result = self.evaluate(config)
        self.assertFalse(result.summary['checks']['minimum_planet_bearing_basic_l10_hours'])
        config.pop('planet_support')
        with self.assertRaises(ValueError):
            self.evaluate(config)

    def test_optional_export_files_report_and_hash_manifest(self):
        result = self.evaluate(self.config())
        with tempfile.TemporaryDirectory() as directory:
            path = export_joint(result, directory)
            self.assertEqual(json.loads((path/'planet_support.json').read_text(encoding='utf-8')), result.planet_support)
            self.assertEqual(json.loads((path/'dc_bus.json').read_text(encoding='utf-8')), result.dc_bus)
            manifest = json.loads((path/'manifest.json').read_text(encoding='utf-8'))
            self.assertIn('planet_support.json', manifest['files'])
            self.assertIn('dc_bus.json', manifest['files'])
            text = (path/'report.md').read_text(encoding='utf-8')
            self.assertIn('行星销与行星轴承', text)
            self.assertIn('直流母线能量初筛', text)
            # 复用同一输出位置时，未配置状态覆盖旧校核，不能残留为本次已计算结果。
            export_joint(self.actual, directory)
            self.assertIsNone(json.loads((path/'planet_support.json').read_text(encoding='utf-8')))
            self.assertIsNone(json.loads((path/'dc_bus.json').read_text(encoding='utf-8')))

    def test_unknown_or_incomplete_optional_parameters_are_rejected(self):
        config = self.config()
        config['planet_support']['unexpected'] = 1
        with self.assertRaises(ValueError):self.evaluate(config)
        config = self.config()
        config['dc_bus'].pop('capacitance_f')
        with self.assertRaises(ValueError):self.evaluate(config)


if __name__ == '__main__':
    unittest.main()
