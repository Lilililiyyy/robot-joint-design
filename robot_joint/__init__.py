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

__version__ = '0.2.0'
