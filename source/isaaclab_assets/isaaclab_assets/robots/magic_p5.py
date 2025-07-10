

from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
import os

##
# Configuration
##

# init_joint_pos = {}

# joint_list =  ['JOINT_HIP_ROLL_L', 'JOINT_HIP_ROLL_R', 'joint_wr', 'JOINT_HIP_YAW_L', 'JOINT_HIP_YAW_R', 'joint_wy', 'JOINT_HIP_PITCH_L', 'JOINT_HIP_PITCH_R', 'joint_hy', 'joint_la1', 'joint_ra1', 'JOINT_KNEE_PITCH_L', 'JOINT_KNEE_PITCH_R', 'joint_hp', 'joint_la2', 'joint_ra2', 'JOINT_ANKLE_PITCH_L', 'JOINT_ANKLE_PITCH_R', 'joint_la3', 'joint_ra3', 'JOINT_ANKLE_ROLL_L', 'JOINT_ANKLE_ROLL_R', 'joint_la4', 'joint_ra4', 'joint_la5', 'joint_ra5', 'joint_la6', 'joint_ra6', 'joint_la7', 'joint_ra7', 'L_thumb_proximal_yaw_joint', 'L_thumb_proximal_pitch_joint', 'L_index_proximal_joint',
#                'L_middle_proximal_joint', 'L_ring_proximal_joint', 'L_pinky_proximal_joint', 
#                 'R_thumb_proximal_yaw_joint', 'R_thumb_proximal_pitch_joint', 'R_index_proximal_joint', 'R_middle_proximal_joint', 'R_ring_proximal_joint',  'R_pinky_proximal_joint',
#                ]

# joint_list =  ['joint_la1', 'joint_la2', 'joint_la3', 'joint_la4', 'joint_la5', 'joint_la6', 'joint_la7', 'joint_ra1', 'joint_ra2', 'joint_ra3', 'joint_ra4', 'joint_ra5', 'joint_ra6', 'joint_ra7', 'L_thumb_proximal_yaw_joint', 'L_thumb_proximal_pitch_joint','L_thumb_intermediate_joint','L_thumb_distal_joint', 'L_index_proximal_joint','L_index_intermediate_joint',
#  'L_middle_proximal_joint', 'L_middle_intermediate_joint', 'L_ring_proximal_joint', 'L_ring_intermediate_joint', 'L_pinky_proximal_joint', 'L_pinky_intermediate_joint', 'R_thumb_proximal_yaw_joint', 'R_thumb_proximal_pitch_joint', 'R_thumb_intermediate_joint', 'R_thumb_distal_joint','R_index_proximal_joint', 'R_index_intermediate_joint', 'R_middle_proximal_joint', 'R_middle_intermediate_joint', 'R_ring_proximal_joint',  'R_ring_intermediate_joint', 'R_pinky_proximal_joint','R_pinky_intermediate_joint',
#  'joint_wr', 'joint_wy', 'joint_hy', 'joint_hp']

# joint_names =  ['joint_la1', 'joint_la2', 'joint_la3', 'joint_la4', 'joint_la5', 'joint_la6', 'joint_la7', 'joint_ra1', 'joint_ra2', 'joint_ra3', 'joint_ra4', 'joint_ra5', 'joint_ra6', 'joint_ra7', 'L_thumb_proximal_yaw_joint', 'L_thumb_proximal_pitch_joint', 'L_index_proximal_joint',
#  'L_middle_proximal_joint', 'L_ring_proximal_joint', 'L_pinky_proximal_joint', 'R_thumb_proximal_yaw_joint', 'R_thumb_proximal_pitch_joint', 'R_index_proximal_joint', 'R_middle_proximal_joint', 'R_ring_proximal_joint',  'R_pinky_proximal_joint',
#  'joint_wr', 'joint_wy', 'joint_hy', 'joint_hp'
#  ]

# for joint_name in joint_list:
#     init_joint_pos.update({joint_name:0.0})

abs_path_factory = os.path.abspath("assets/Factory")[:-14]  # 获取项目根目录的绝对路径

HUMANOID_MAGIC_P5_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",  # 定义机器人在USD场景中的根路径
    spawn=sim_utils.UsdFileCfg(  # 配置机器人模型加载和物理属性
        usd_path=os.path.join(abs_path_factory, "assets/p5_inspire/p5_robot.usd"),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(  # 刚体物理属性配置
            disable_gravity=None,
            max_depenetration_velocity=1.0,  # 最大穿透解除速度，之前默认设置的是100.0
            enable_gyroscopic_forces=True,  # 启用陀螺效应
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(  # 关节系统配置
            enabled_self_collisions=False,
            solver_position_iteration_count=4,  # 物理位置解算迭代次数（影响精度）
            solver_velocity_iteration_count=4,  # 速度解算迭代次数（影响精度），之前默认设置的是0
            sleep_threshold=0.005,  # 睡眠阈值
            stabilization_threshold=0.001,  # 稳定化阈值
        ),
        copy_from_source=False,  # 直接使用原USD文件（不创建副本）
    ),
    init_state=ArticulationCfg.InitialStateCfg(  # 设置机器人初始状态
        pos=(0.0, 0.0, 0.8),  # 初始位置
        joint_pos={".*": 0.0},  # 初始关节角度
        # joint_pos=init_joint_pos,  # 初始关节角度
        # joint_vel={".*": 0.0},
    ),
    actuators={  # 配置执行器（电机）参数
        "body": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            stiffness=None,  # 使用物理材料的默认刚度
            damping=None,  # 使用物理材料的默认阻尼
        ),
    },
)


