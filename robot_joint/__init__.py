"""NGW机器人关节模组参数化设计计算；SI单位与可复制源码接口。"""
from .planetary import PlanetaryStage, stage_geometry, speed_map, solve_load_sharing
from .gears import gear_pairs, coupled_planet_contacts, CouplingConvergenceError
from .analysis import JointResult, analyze_joint, compare_joint_designs
from .support import support_response, two_bearing_reactions, bearing_duty_life
from .fatigue import rainflow_cycles, sn_damage
from .thermal import thermal_response, periodic_thermal_response
from .electromechanical import motor_duty
from .precision import precision_budget, reversal_curve
from .io import export_joint
from .planet_support import pin_response, assess_planet_support
from .dc_bus import dc_bus_screen
from .contact_pattern import ContactPattern, build_contact_pattern, export_contact_pattern, interior_snapshot
from .contact_display import contact_pass_data, member_contact_pass_data, instantaneous_contact_data, draw_contact_pass

__version__ = '0.3.2'
