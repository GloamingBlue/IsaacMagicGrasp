

from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

##
# Configuration
##
init_joint_pos = {}

joint_list =  ['JOINT_HIP_ROLL_L', 'JOINT_HIP_ROLL_R', 'joint_wr', 'JOINT_HIP_YAW_L', 'JOINT_HIP_YAW_R', 'joint_wy', 'JOINT_HIP_PITCH_L', 'JOINT_HIP_PITCH_R', 'joint_hy', 'joint_la1', 'joint_ra1', 'JOINT_KNEE_PITCH_L', 'JOINT_KNEE_PITCH_R', 'joint_hp', 'joint_la2', 'joint_ra2', 'JOINT_ANKLE_PITCH_L', 'JOINT_ANKLE_PITCH_R', 'joint_la3', 'joint_ra3', 'JOINT_ANKLE_ROLL_L', 'JOINT_ANKLE_ROLL_R', 'joint_la4', 'joint_ra4', 'joint_la5', 'joint_ra5', 'joint_la6', 'joint_ra6', 'joint_la7', 'joint_ra7', 'L_thumb_proximal_yaw_joint', 'L_thumb_proximal_pitch_joint', 'L_index_proximal_joint',
               'L_middle_proximal_joint', 'L_ring_proximal_joint', 'L_pinky_proximal_joint', 
                'R_thumb_proximal_yaw_joint', 'R_thumb_proximal_pitch_joint', 'R_index_proximal_joint', 'R_middle_proximal_joint', 'R_ring_proximal_joint',  'R_pinky_proximal_joint',
               ]

joint_list =  ['joint_la1', 'joint_la2', 'joint_la3', 'joint_la4', 'joint_la5', 'joint_la6', 'joint_la7', 'joint_ra1', 'joint_ra2', 'joint_ra3', 'joint_ra4', 'joint_ra5', 'joint_ra6', 'joint_ra7', 'L_thumb_proximal_yaw_joint', 'L_thumb_proximal_pitch_joint','L_thumb_intermediate_joint','L_thumb_distal_joint', 'L_index_proximal_joint','L_index_intermediate_joint',
 'L_middle_proximal_joint', 'L_middle_intermediate_joint', 'L_ring_proximal_joint', 'L_ring_intermediate_joint', 'L_pinky_proximal_joint', 'L_pinky_intermediate_joint', 'R_thumb_proximal_yaw_joint', 'R_thumb_proximal_pitch_joint', 'R_thumb_intermediate_joint', 'R_thumb_distal_joint','R_index_proximal_joint', 'R_index_intermediate_joint', 'R_middle_proximal_joint', 'R_middle_intermediate_joint', 'R_ring_proximal_joint',  'R_ring_intermediate_joint', 'R_pinky_proximal_joint','R_pinky_intermediate_joint',
 'joint_wr', 'joint_wy', 'joint_hy', 'joint_hp']

# joint_names =  ['joint_la1', 'joint_la2', 'joint_la3', 'joint_la4', 'joint_la5', 'joint_la6', 'joint_la7', 'joint_ra1', 'joint_ra2', 'joint_ra3', 'joint_ra4', 'joint_ra5', 'joint_ra6', 'joint_ra7', 'L_thumb_proximal_yaw_joint', 'L_thumb_proximal_pitch_joint', 'L_index_proximal_joint',
#  'L_middle_proximal_joint', 'L_ring_proximal_joint', 'L_pinky_proximal_joint', 'R_thumb_proximal_yaw_joint', 'R_thumb_proximal_pitch_joint', 'R_index_proximal_joint', 'R_middle_proximal_joint', 'R_ring_proximal_joint',  'R_pinky_proximal_joint',
#  'joint_wr', 'joint_wy', 'joint_hy', 'joint_hp'
#  ]

for joint_name in joint_list:
    init_joint_pos.update({joint_name:0.0})


HUMANOID_MAGIC_P5_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(
        # usd_path="/mnt/new_space/code/Wiki-MJCF-master/models/GR1/GR1T1/urdf/GR1T1/GR1T1.usd",
        usd_path="assets/p5_inspire/p5_robot.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=None,
            max_depenetration_velocity=100.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
        copy_from_source=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.8),
        joint_pos=init_joint_pos,
    ),
    actuators={
        "body": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            stiffness=None,
            damping=None,
        ),
    },
)


