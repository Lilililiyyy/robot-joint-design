"""销的弯剪能量、虚功、尺度律及行星轴承相对转速的独立验证。"""
from copy import deepcopy
import math
import unittest
import numpy as np

from robot_joint.planet_support import pin_response, assess_planet_support


class PinResponseTests(unittest.TestCase):
    def options(self):
        return dict(diameter_m=.004, support_span_m=.012, loaded_width_m=.005,
                    young_modulus_pa=210e9, poisson_ratio=.3, shear_coefficient=.9,
                    allowable_bending_stress_pa=300e6, allowable_shear_stress_pa=180e6,
                    allowable_projected_pressure_pa=60e6, required_safety_factor=1.5)

    def test_energy_and_center_virtual_work_against_piecewise_gauss_integrals(self):
        p = self.options()
        result = pin_response([30, -40], **p)
        force = 50.
        length, width, d = p['support_span_m'], p['loaded_width_m'], p['diameter_m']
        a = (length-width)/2
        ei = p['young_modulus_pa']*math.pi*d**4/64
        kga = p['shear_coefficient']*p['young_modulus_pa']/(2*(1+p['poisson_ratio']))*math.pi*d*d/4
        nodes, weights = np.polynomial.legendre.leggauss(8)
        energy, center = 0., 0.
        # 每半跨按载荷起点拆段；数值积分公式与生产代码闭式多项式独立。
        for low, high in ((0., a), (a, length/2)):
            x = low+(nodes+1)*(high-low)/2
            z = np.maximum(0., x-a)
            moment = force*x/2-force*z*z/(2*width)
            shear = force/2-force*z/width
            energy += float(np.dot(weights, moment**2/ei+shear**2/kga))*(high-low)/2
            center += 2*float(np.dot(weights, moment*(x/2)/ei+shear*.5/kga))*(high-low)/2
        self.assertAlmostEqual(result['strain_energy_j'], energy, places=15)
        self.assertAlmostEqual(result['load_weighted_deflection_m'], 2*energy/force, places=15)
        self.assertAlmostEqual(result['center_deflection_m'], center, places=15)

    def test_full_span_uniform_load_closed_form(self):
        p = self.options()
        p['loaded_width_m'] = p['support_span_m']
        result = pin_response([100, 0], **p)
        length, d = p['support_span_m'], p['diameter_m']
        ei = p['young_modulus_pa']*math.pi*d**4/64
        kga = p['shear_coefficient']*p['young_modulus_pa']/(2*(1+p['poisson_ratio']))*math.pi*d*d/4
        self.assertAlmostEqual(result['maximum_bending_moment_n_m'], 100*length/8)
        self.assertAlmostEqual(result['center_deflection_m'], 5*100*length**3/(384*ei)+100*length/(8*kga), places=15)
        self.assertAlmostEqual(result['load_weighted_deflection_m'], 100*length**3/(120*ei)+100*length/(12*kga), places=15)

    def test_point_load_limit(self):
        p = self.options()
        p['loaded_width_m'] = p['support_span_m']*1e-8
        result = pin_response([10, 0], **p)
        length, d = p['support_span_m'], p['diameter_m']
        ei = p['young_modulus_pa']*math.pi*d**4/64
        kga = p['shear_coefficient']*p['young_modulus_pa']/(2*(1+p['poisson_ratio']))*math.pi*d*d/4
        expected = 10*length**3/(48*ei)+10*length/(4*kga)
        self.assertAlmostEqual(result['center_deflection_m']/expected, 1., places=7)
        self.assertAlmostEqual(result['load_weighted_deflection_m']/expected, 1., places=7)

    def test_force_reversal_rotation_scaling_and_no_false_combined_stress(self):
        first = pin_response([30, -40], **self.options())
        opposite = pin_response([-30, 40], **self.options())
        rotated = pin_response([0, 50], **self.options())
        doubled = pin_response([60, -80], **self.options())
        for key in ('bending_stress_pa', 'maximum_transverse_shear_stress_pa', 'projected_average_pressure_pa', 'center_deflection_m'):
            self.assertAlmostEqual(first[key], opposite[key])
            self.assertAlmostEqual(first[key], rotated[key])
            self.assertAlmostEqual(doubled[key], 2*first[key])
        np.testing.assert_allclose(first['center_displacement_xy_m'], -np.array(opposite['center_displacement_xy_m']))
        np.testing.assert_allclose(np.sum(first['support_reactions_xy_n'], axis=0), [-30, 40])
        self.assertNotIn('von_mises_stress_pa', first)

    def test_diameter_scaling_separates_bending_and_shear(self):
        p = self.options()
        first = pin_response([100, 0], **p)
        p['diameter_m'] *= 2
        larger = pin_response([100, 0], **p)
        self.assertAlmostEqual(first['bending_stress_pa']/larger['bending_stress_pa'], 8.)
        self.assertAlmostEqual(first['maximum_transverse_shear_stress_pa']/larger['maximum_transverse_shear_stress_pa'], 4.)
        self.assertAlmostEqual(first['load_weighted_bending_compliance_m_per_n']/larger['load_weighted_bending_compliance_m_per_n'], 16.)
        self.assertAlmostEqual(first['load_weighted_shear_compliance_m_per_n']/larger['load_weighted_shear_compliance_m_per_n'], 4.)

    def test_zero_load_has_defined_stiffness_and_undefined_safety_ratio(self):
        result = pin_response([0, 0], **self.options())
        self.assertEqual(result['load_weighted_deflection_m'], 0)
        self.assertGreater(result['equivalent_pin_stiffness_n_per_m'], 0)
        self.assertTrue(result['static_checks_pass'])
        self.assertIsNone(result['checks']['bending']['safety_factor'])

    def test_actual_strength_failure_and_invalid_inputs(self):
        p = self.options()
        self.assertFalse(pin_response([1e6, 0], **p)['static_checks_pass'])
        for key, value in (('diameter_m', 0), ('loaded_width_m', .02), ('poisson_ratio', .5),
                           ('shear_coefficient', 1.1), ('required_safety_factor', .9),
                           ('support_model', 'cantilever'), ('young_modulus_pa', True)):
            q = dict(p);q[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                pin_response([10, 0], **q)
        for force in ([True, False], [1], [math.nan, 0], ['1', '2']):
            with self.assertRaises(ValueError):
                pin_response(force, **p)


class PlanetDutyTests(unittest.TestCase):
    def setup_duty(self):
        cases = [{'label': 'forward', 'sharing': {'carrier_pin_forces_xy_n': [[300., -400.], [0., 250.]]},
                  'speeds': {'planet_relative_to_carrier_rad_s': 20*math.pi, 'planet_rad_s': math.pi}},
                 {'label': 'reverse', 'sharing': {'carrier_pin_forces_xy_n': [[-300., 400.], [0., -250.]]},
                  'speeds': {'planet_relative_to_carrier_rad_s': -20*math.pi, 'planet_rad_s': -math.pi}}]
        segments = [{'duration_s': 1800.}, {'duration_s': 1800.}]
        rating = {'dynamic_capacity_n': 5000., 'static_capacity_n': 2000., 'exponent': 3.,
                  'radial_factor': 1., 'axial_factor': 0., 'static_radial_factor': 1., 'static_axial_factor': 0.}
        return cases, segments, rating

    def test_relative_speed_rating_and_no_mutation_or_feedback(self):
        cases, segments, rating = self.setup_duty()
        before = deepcopy(cases)
        result = assess_planet_support(cases, segments, pins=PinResponseTests().options(),
                                       bearing_ratings=rating, specified_pin_stiffness_n_per_m=[3e8, 3e8])
        self.assertEqual(cases, before)
        self.assertFalse(result['summary']['system_stiffness_was_updated'])
        self.assertAlmostEqual(result['planet_bearings'][0]['life_hours'], 1e6/(600*60)*10**3)
        self.assertAlmostEqual(result['planet_bearings'][0]['minimum_static_safety_factor'], 4.)
        self.assertAlmostEqual(result['planet_bearings'][1]['life_hours']/result['planet_bearings'][0]['life_hours'], 8.)
        self.assertAlmostEqual(result['pins'][0]['stiffness_ratio_to_system_input'],
                               result['pins'][0]['equivalent_pin_stiffness_n_per_m']/3e8)

    def test_stationary_bearing_has_static_result_but_no_l10(self):
        cases, segments, rating = self.setup_duty()
        for case in cases:case['speeds']['planet_relative_to_carrier_rad_s'] = 0.
        result = assess_planet_support(cases, segments, pins=PinResponseTests().options(), bearing_ratings=rating)
        self.assertIsNone(result['planet_bearings'][0]['life_hours'])
        self.assertAlmostEqual(result['planet_bearings'][0]['minimum_static_safety_factor'], 4.)
        self.assertIsNone(result['pins'][0]['stiffness_ratio_to_system_input'])

    def test_missing_branch_data_or_changed_layout_is_rejected(self):
        cases, segments, rating = self.setup_duty()
        missing = dict(rating);missing.pop('radial_factor')
        with self.assertRaises(ValueError):
            assess_planet_support(cases, segments, pins=PinResponseTests().options(), bearing_ratings=missing)
        cases[1]['sharing']['carrier_pin_forces_xy_n'] = [[1., 2.]]
        with self.assertRaises(ValueError):
            assess_planet_support(cases, segments, pins=PinResponseTests().options(), bearing_ratings=rating)


if __name__ == '__main__':
    unittest.main()
