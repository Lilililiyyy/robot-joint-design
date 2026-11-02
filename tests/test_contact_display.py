"""新写显示接口测试：独立坐标关系、原载荷保持及带身份校验的重绘。"""
from copy import deepcopy
from pathlib import Path
import math
import tempfile
import unittest
import numpy as np

from gear_contact import analyze_config
from gear_contact.io import export_result
from robot_joint.contact_display import (contact_pass_data, member_contact_pass_data,
    instantaneous_contact_data, load_plot_result)


def config(internal=False):
    gear=dict(normal_module_m=.0005,face_width_m=.005)
    return dict(pair=dict(pinion=dict(teeth=20,**gear),
        wheel=dict(teeth=60 if internal else 40,internal=internal,**gear)),
        phases=32,strips=9,torque_n_m=.3,elastic_mode='iso',local_contact=False,
        iso_settings=dict(nominal_pinion_torque_n_m=.3),
        modification_pinion=dict(profile_crown_m=3e-6,lead_crown_m=.3e-6))


class ContactDisplayTests(unittest.TestCase):
    def test_wheel_face_origin_accounts_for_unequal_width_and_axial_offset(self):
        data=config()
        data['pair']['pinion']['face_width_m']=.010
        data['pair']['wheel']['face_width_m']=.008
        data['pair']['axial_offset_m']=.002
        result=analyze_config(data)
        wheel=member_contact_pass_data(result,member='wheel')
        # 大轮全局区间[-2,6]mm，小轮[-5,5]mm，重合区在大轮左端起为[0,7]mm。
        np.testing.assert_allclose(wheel['face_coordinate_m'][[0,-1]],[0,.007],atol=1e-12)

    def test_external_and_internal_conjugate_coordinates_preserve_pressure(self):
        for internal in (False,True):
            result=analyze_config(config(internal))
            pinion=member_contact_pass_data(result)
            wheel=member_contact_pass_data(result,member='wheel')
            span=result.pair.center_distance_m*math.sin(result.pair.working_pressure_angle_rad)
            sign=1 if internal else -1
            expected=np.sort(span+sign*pinion['roll_distance_m'])
            np.testing.assert_allclose(wheel['roll_distance_m'],expected,rtol=1e-12,atol=1e-12)
            np.testing.assert_allclose(wheel['samples'][:,1],span+sign*pinion['samples'][:,1])
            expected_field=pinion['pressure_pa_x_y'] if internal else pinion['pressure_pa_x_y'][:,::-1]
            np.testing.assert_allclose(wheel['pressure_pa_x_y'],expected_field,equal_nan=True)

    def test_local_hertz_band_retains_source_force_and_micrometre_width(self):
        result=analyze_config(config())
        local=instantaneous_contact_data(result,normal_points=1001)
        phase,tooth=local['phase_index'],local['tooth_pair_index']
        mask=result.rotations[phase].grid.tooth_pair_index==tooth
        np.testing.assert_array_equal(local['source_force_n'],result.rotations[phase].solution.forces_n[mask])
        recovered=math.pi/2*local['source_pressure_pa']*local['source_half_width_m']*local['source_widths_m']
        np.testing.assert_allclose(recovered,local['source_force_n'],rtol=1e-10,atol=1e-10)
        raster=np.sum(local['pressure_pa_s_n']*np.diff(local['s_edges_m'])[:,None]*np.diff(local['normal_edges_m'])[None,:])
        self.assertAlmostEqual(raster/sum(local['source_force_n']),1.,delta=5e-4)
        self.assertGreater(np.ptp(local['s_edges_m'])/np.ptp(local['normal_edges_m']),100)
        self.assertFalse(local['independent_2d_contact'])

    def test_saved_arrays_reproduce_live_display_and_reject_identity_change(self):
        result=analyze_config(config())
        expected=contact_pass_data(result)
        with tempfile.TemporaryDirectory() as folder:
            export_result(result,folder)
            loaded=load_plot_result(folder)
            actual=contact_pass_data(loaded)
            np.testing.assert_allclose(actual['pressure_pa_x_y'],expected['pressure_pa_x_y'],equal_nan=True)
            np.testing.assert_array_equal(actual['samples'],expected['samples'])
            np.testing.assert_allclose(instantaneous_contact_data(loaded)['pressure_pa_s_n'],
                                       instantaneous_contact_data(result)['pressure_pa_s_n'])
            self.assertTrue(loaded.plot_only)
            with (Path(folder)/'arrays.npz').open('ab') as stream:
                stream.write(b'changed')
            with self.assertRaisesRegex(ValueError,'身份'):
                load_plot_result(folder)

    def test_local_coordinate_arguments_do_not_silently_select_other_phase(self):
        result=analyze_config(config())
        for kwargs in (dict(phase_index=-1),dict(phase_index=True),dict(normal_points=1),
                       dict(phase_index=0,tooth_pair_index=999)):
            with self.assertRaises(ValueError):
                instantaneous_contact_data(result,**kwargs)


if __name__=='__main__':unittest.main()
