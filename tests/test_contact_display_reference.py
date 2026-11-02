"""固定接触显示场回归及直齿分载关系；不对斑点外形拟合。"""
from base64 import b64decode
from copy import deepcopy
from io import BytesIO
import math
import unittest

import numpy as np

from gear_contact import analyze_config
from robot_joint.contact_display import contact_pass_data


# 保留完整分析输入，含实际形径、组合修形、延伸、相位与名义/LTCA扭矩。
HELICAL_REFERENCE_CONFIG = {'pair': {'pinion': {'teeth': 18,
                     'normal_module_m': 0.002,
                     'face_width_m': 0.024,
                     'pressure_angle_rad': 0.39269908169872414,
                     'helix_angle_rad': -0.41015237421866746,
                     'profile_shift': 0.32679994951588004,
                     'internal': False,
                     'tip_diameter_m': 0.04456307954415094,
                     'root_diameter_m': 0.03556307954415094,
                     'root_form_diameter_m': 0.036887473786745395,
                     'tip_form_diameter_m': 0.04456307954415094},
          'wheel': {'teeth': 77,
                    'normal_module_m': 0.002,
                    'face_width_m': 0.024,
                    'pressure_angle_rad': 0.39269908169872414,
                    'helix_angle_rad': 0.41015237421866746,
                    'profile_shift': -0.3267999495158864,
                    'internal': False,
                    'tip_diameter_m': 0.1706207302268636,
                    'root_diameter_m': 0.1616207302268636,
                    'root_form_diameter_m': 0.1628285581981177,
                    'tip_form_diameter_m': 0.1706207302268636},
          'center_distance_m': None,
          'axial_offset_m': 0.0},
 'material_pinion': {'youngs_modulus_pa': 207000000000, 'poisson_ratio': 0.3},
 'material_wheel': {'youngs_modulus_pa': 207000000000, 'poisson_ratio': 0.3},
 'modification_pinion': {'kind': 'composite',
                         'components': [{'kind': 'parameters',
                                         'profile_crown_m': 8.230316576566792e-07,
                                         'lead_crown_m': 4.874995545711296e-07},
                                        {'kind': 'curve',
                                         'axis': 'profile',
                                         'coordinates': [0.0, 1.0],
                                         'removal_m': [-1.5881867761018131e-22,
                                                       -3.970466940254533e-22]},
                                        {'kind': 'curve',
                                         'axis': 'lead',
                                         'coordinates': [0.0, 1.0],
                                         'removal_m': [0.0, -1.5881867761018131e-22]},
                                        {'kind': 'curve',
                                         'axis': 'lead',
                                         'bounds': 'extrapolate',
                                         'coordinates': [0.09999999999999998,
                                                         0.30000000000000004,
                                                         0.5,
                                                         0.7,
                                                         0.8999999999999999],
                                         'removal_m': [4.6987842879276434e-08,
                                                       2.8867652361613518e-08,
                                                       9.630013187570531e-09,
                                                       -2.4891169218597974e-08,
                                                       -6.059433920986232e-08]}]},
 'modification_wheel': {'kind': 'composite',
                        'components': [{'kind': 'parameters',
                                        'profile_crown_m': 0.0,
                                        'lead_crown_m': 0.0},
                                       {'kind': 'curve',
                                        'axis': 'profile',
                                        'coordinates': [0.0, 1.0],
                                        'removal_m': [0.0, 0.0]},
                                       {'kind': 'curve',
                                        'axis': 'lead',
                                        'coordinates': [0.0, 1.0],
                                        'removal_m': [0.0, 0.0]}]},
 'phases': 32,
 'strips': 40,
 'elastic_mode': 'iso_love',
 'local_contact': False,
 'iso_settings': {'application_factor': 1.0,
                  'material_reference_youngs_modulus_pa': 207000000000.0,
                  'nominal_pinion_torque_n_m': 1.718873394980122},
 'extended_contact_settings': {'face_extension_factor': 0.14073562622070318,
                               'roll_extension_factor': 0.4},
 'phase_offset': 0.036931454775936845,
 'torque_n_m': 2.5361648083548416,
 'misalignment_m': 0.0}

# 用标准NPZ冻结数值，测试可随独立项目复制；加载显式禁用pickle。
EXPECTED_DISPLAY_NPZ_BASE64 = (
    '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'
    'SzJIT2MNhagJJoJJz0nfdG08v5rRyBrjqPz2DA25Ky5nlUpovHm9w9W7lkbcw/6r+TU06E0PFPiXafiv3L7Lj3xutr3tLpO1NPSN'
    'sla+kaRRVVMSvKGIQkqky48aFwoPtuQ94vUJIefe1saeFGJ4+ISbE/4KoDiwpmdtOR+rg1qCpdfzYJ3VXKCWlQmhlJ6zf1k6bAKl'
    '048lpkIYrXwhsjsJxgf+CHNvxsOb/+xZqeTZIXg/5LqJXSozoTVvvqimlo8vlg+11MOFsDn+gfP1oKD+KsgxbDSNGsMnuRGRNEp1'
    'FFSXddIImGdem67FQFL6Xl3AWgZNFokefEcGGc2z69xtGQTKlNUFL2RwxXL84biRDG49nlikSOrxlJuuOhBPw7Vs249d1jTMHi9c'
    'rtFL+q+hOLIpgcKGrDc2mzUoGMWNYHUThRhHK8u8fS+Aq63xI4UgPq6v36nx+V0WSvTE/Q9HZqJzVUdFUXsa2qprX2vuSIG2mcyT'
    'YTsSoc5clrTbFjN0f+0aKu/38rD6Z+fcz6sIryi08GUcKBzeNv3AJBUa5TLZpmcv0Qi9WXMikOAb73p/XPthBpqHLolnXGJQu3vP'
    'xS/VDN5fWKp7oZ5B7ndbhxvlDNpP2kU7sQyMxaX+ZUcZKGv41h4jdYlLc/icpsjAoCZ2Wem/NCamP5rlGUTjoMlLqYzJNJKWLHgr'
    'pinU7lSYrq5EwT7O74m2txCrPy6jhJECqAzT+PKkk4crdxf+1lDMAutqpF0xLANsm6vKYHQqxGG346SuJ8E6dfJkn7XxQ/CCS814'
    'f0EAx+8piy5OoGBr/Tjp5S8KCpf6It5doWFt9oW5uYxBuMZNle5cBp9mV6Up9TKwtFf6lDuNhZ4B3bDGiMVhmboPRqYsHF7EFs/W'
    'ZnEt3lSrWI6F71mPd/VdDNJSXj3eU8HAtVBqW+tZBk/O3ExVXMXAPXd/lWEvjY9vDIddjKBhGf+oTFKOhnh23wz7UArnOuL/vfdW'
    'CEXFPp0NBkLU3HKwNa/ko/RXQmvFAOlzhFWrwbsM1JUahXueScPW/sozUo+TYex6RcopOWEI3sAxIVVXU4TI3zVu9N1LFDblJDuJ'
    'o2lUf6u73G/KIL02ZmLxDQaDpzqPNKmyeP7o77lzTiwOTcmZ8OM4i5Zq65mbU1hczz5nzksgeBM0I90iWfQGlcfNPcLC7YpfR5Ij'
    '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'
)


def spur_config():
    """原负向峰载太阳副planet2；总法向力约63.2593N，m/N/Pa/rad。"""
    gear = dict(normal_module_m=.0005, face_width_m=.005)
    return dict(pair=dict(pinion=dict(teeth=20, **gear), wheel=dict(teeth=40, **gear)),
        torque_n_m=.2972214072205919, phases=128, strips=21,
        elastic_mode='iso', local_contact=False,
        iso_settings=dict(nominal_pinion_torque_n_m=.2972214072205919),
        modification_pinion=dict(lead_crown_m=.3e-6),
        modification_wheel=dict(lead_crown_m=.2e-6))


def sorted_line_forces(result):
    """相同齿材料路径按小轮展长拼接，只读原条带力而不读取显示插值。"""
    rows = []
    for rotation in result.rotations:
        grid = rotation.grid
        for tooth in np.unique(grid.tooth_pair_index):
            mask = grid.tooth_pair_index == tooth
            rows.append((float(np.mean(grid.roll_pinion_m[mask])),
                         float(np.sum(rotation.solution.forces_n[mask]))))
    return np.asarray(sorted(rows))[:, 1]


class ContactDisplayReferenceTests(unittest.TestCase):
    def test_endpoint_band_reports_display_integral_difference_without_rescaling(self):
        from robot_joint.contact_display import instantaneous_contact_data
        result = analyze_config(deepcopy(HELICAL_REFERENCE_CONFIG))
        before = result.rotations[0].endpoint_pressure['pressure_pa'].copy()
        local = instantaneous_contact_data(result, phase_index=0, tooth_pair_index=0, normal_points=3001)
        # 原中心力与端点p0/b线性显示面积不是同一离散量；保留差异，禁止图形归一化。
        self.assertAlmostEqual(local['source_total_force_n'], 104.67288793968982, places=7)
        self.assertAlmostEqual(local['display_integrated_force_n'], 104.46356565130891, places=6)
        self.assertAlmostEqual(local['display_relative_force_error'], -.0019997756104857853, places=9)
        np.testing.assert_array_equal(result.rotations[0].endpoint_pressure['pressure_pa'], before)
        self.assertIn('source_endpoint_pressure_pa', local)

    def test_same_18_77_result_matches_expected_30_by_30_grid_and_nan_mask(self):
        result = analyze_config(deepcopy(HELICAL_REFERENCE_CONFIG))
        with np.load(BytesIO(b64decode(EXPECTED_DISPLAY_NPZ_BASE64)), allow_pickle=False) as source:
            x, y = source['face_coordinate_m'], source['roll_distance_m']
            expected = source['pressure_pa_x_y']
            actual = contact_pass_data(result, face_coordinate_m=x, roll_distance_m=y)
        self.assertEqual(actual['pressure_pa_x_y'].shape, (30, 30))
        np.testing.assert_array_equal(actual['face_coordinate_m'], x)
        np.testing.assert_array_equal(actual['roll_distance_m'], y)
        np.testing.assert_array_equal(np.isnan(actual['pressure_pa_x_y']), np.isnan(expected))
        np.testing.assert_allclose(actual['pressure_pa_x_y'], expected, rtol=1e-10,
                                   atol=.01, equal_nan=True)
        self.assertEqual(len(actual['samples']), 5166)
        self.assertLess(len(actual['ordinary_samples']), len(actual['samples']))

    def test_iso_load_jump_persists_with_love_and_profile_crown_changes_loading(self):
        baseline = spur_config()
        love = deepcopy(baseline)
        love['elastic_mode'] = 'iso_love'
        crown = deepcopy(love)
        for member in ('modification_pinion', 'modification_wheel'):
            crown[member]['profile_crown_m'] = 3e-6
        results = [analyze_config(c) for c in (baseline, love, crown)]
        jumps = []
        for result in results:
            force = sorted_line_forces(result)
            total = result.summary['normal_force_n']
            jumps.append(float(np.max(abs(np.diff(force))) / total))
            for rotation in result.rotations:
                actual = rotation.solution.forces_n
                self.assertAlmostEqual(float(np.sum(actual)), total, places=10)
                recovered = (math.pi/2 * rotation.pressure_pa * rotation.half_width_m
                             * rotation.grid.widths_m)
                np.testing.assert_allclose(recovered, actual, rtol=2e-10, atol=1e-10)
        self.assertAlmostEqual(jumps[0], .5, places=10)
        self.assertAlmostEqual(jumps[1], .5, places=10)
        self.assertLess(jumps[2], .03)
        crown_forces = sorted_line_forces(results[2])
        self.assertEqual(crown_forces[0], 0.)
        self.assertEqual(crown_forces[-1], 0.)
        # 诊断候选有接近量代价；这不是最优修形或制造推荐值。
        self.assertGreater(np.mean([r.solution.approach_m for r in results[2].rotations]),
                           np.mean([r.solution.approach_m for r in results[0].rotations]))


if __name__ == '__main__':
    unittest.main()
