"""由公开摆台参数生成再设计工况；运动轨迹和未公开参数均为显式设计输入。"""
from copy import deepcopy
import math


def validate_saved_duty(generated,saved):
    """按明确数值容差校验保存工况，避免跨平台libm末位差异误报。

    标签、字段集合、顺序和六分量布局必须一致。有限SI数值采用
    rtol=2e-12、atol=2e-12；实际输入变更不能借此静默通过。
    """
    if len(generated)!=len(saved):raise ValueError('保存工况数量与需求不一致')
    for index,(actual,expected) in enumerate(zip(generated,saved)):
        if set(actual)!=set(expected):raise ValueError(f'工况{index}字段集合不一致')
        for key,value in actual.items():
            reference=expected[key]
            if key=='label':
                if value!=reference:raise ValueError(f'工况{index}标签不一致')
                continue
            a,b=(value,reference) if key=='output_wrench' else ([value],[reference])
            if key=='output_wrench' and (len(a)!=6 or len(b)!=6):
                raise ValueError('输出载荷必须为六分量')
            for x,y in zip(a,b):
                if (isinstance(x,bool) or isinstance(y,bool) or not math.isfinite(x) or not math.isfinite(y)
                        or not math.isclose(x,y,rel_tol=2e-12,abs_tol=2e-12)):
                    raise ValueError(f'工况{index}字段{key}与需求公式不一致')


def pendulum_duty(requirements):
    """按平面刚体逆动力学生成往返五次轨迹和两端保持段。

    点质量位于L，杆件按均匀细杆处理，转轴沿z，θ=0竖直向下。
    I=mp L²+mr L²/3；T=I α+g(mp L+mr L/2)sinθ+bω。
    公开COM为约0.45m；本模型从质量分布推导0.451923m，保留取整差异。
    各段用中点代表值进入准静态内核；端点和密集曲线另存用于离散核验。
    """
    p = requirements['pendulum']
    duty = requirements['motion_design']
    mp, mr, length, gravity = p['point_mass_kg'], p['rod_mass_kg'], p['length_m'], p['gravity_m_per_s2']
    mass = mp+mr
    inertia = (mp+mr/3)*length**2
    first_moment = (mp+mr/2)*length
    com = first_moment/mass
    amplitude = duty['amplitude_rad']
    move_time, hold_time = duty['move_duration_s'], duty['hold_duration_s']
    count = duty['samples_per_move']
    if (isinstance(count, bool) or not isinstance(count, int) or count < 4
            or move_time <= 0 or hold_time <= 0 or not 0 < amplitude < math.pi):
        raise ValueError('往返工况的时间、幅值和采样数无效')
    segments, states = [], []
    offset = 0.0
    for phase, start, end in [('forward',-amplitude,amplitude), ('reverse',amplitude,-amplitude)]:
        for j in range(count):
            u = (j+.5)/count
            q = 10*u**3-15*u**4+6*u**5
            dq = 30*u**2-60*u**3+30*u**4
            ddq = 60*u-180*u**2+120*u**3
            theta = start+(end-start)*q
            speed = (end-start)*dq/move_time
            acceleration = (end-start)*ddq/move_time**2
            torque = inertia*acceleration+gravity*first_moment*math.sin(theta)+duty['viscous_damping_n_m_s_per_rad']*speed
            ax = com*(acceleration*math.cos(theta)-speed**2*math.sin(theta))
            ay = com*(acceleration*math.sin(theta)+speed**2*math.cos(theta))
            segments.append(dict(label=f'{phase}_{j+1:02d}',duration_s=move_time/count,
                output_torque_n_m=torque,output_speed_rad_s=speed,output_acceleration_rad_s2=acceleration,
                output_wrench=[mass*ax,mass*(ay+gravity),0,0,0,torque]))
            states.append(dict(time_s=offset+(j+.5)*move_time/count,theta_rad=theta,
                               speed_rad_s=speed,acceleration_rad_s2=acceleration))
        offset += move_time
        torque = gravity*first_moment*math.sin(end)
        segments.append(dict(label='hold_'+phase,duration_s=hold_time,output_torque_n_m=torque,
            output_speed_rad_s=0,output_acceleration_rad_s2=0,output_wrench=[0,mass*gravity,0,0,0,torque]))
        states.append(dict(time_s=offset+hold_time/2,theta_rad=end,speed_rad_s=0,acceleration_rad_s2=0))
        offset += hold_time
    return segments, {'total_mass_kg':mass,'inertia_kg_m2':inertia,'derived_com_m':com,
                      'gravity_first_moment_n_m':gravity*first_moment,'period_s':offset,'states':states}


def candidate_configs(base):
    """保持几何、材料和运动需求相同，只比较显式误差与销柔度。"""
    configs = [deepcopy(base) for _ in range(3)]
    names = ['baseline','larger_errors','flexible_pins']
    for config, name in zip(configs,names):
        config['case_id'] = 'dfki_pendulum_'+name
    for key in ('positive_path_gaps_m','negative_path_gaps_m'):
        configs[1]['load_sharing'][key] = [3*v for v in base['load_sharing'][key]]
    configs[2]['load_sharing']['pin_stiffness_n_per_m'] = [v*.3 for v in base['load_sharing']['pin_stiffness_n_per_m']]
    return configs
