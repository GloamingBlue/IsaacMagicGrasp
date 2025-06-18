import argparse
# from pxr import UsdGeom, UsdPhysics
from isaaclab.app import AppLauncher
import torch
import numpy as np
import math
from random import random, uniform, shuffle
from scipy.spatial.transform import Rotation as R
# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on using the differential IK controller.")
parser.add_argument("--robot", type=str, default="magic_p5", help="Name of the robot.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)

import isaaclab.utils.math as math_utils
import isaaclab.sim as sim_utils
from isaacsim.core.prims import XFormPrim
from isaaclab.sim import PinholeCameraCfg
from isaaclab.sensors.camera import Camera, CameraCfg
# from isaaclab.sensors.camera_cfg import PinholeCameraCfg
from isaaclab.sim.spawners.sensors.sensors_cfg import PinholeCameraCfg
from isaaclab.assets import Articulation, RigidObject, AssetBaseCfg, RigidObjectCfg
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.markers import VisualizationMarkers
from isaaclab.markers.config import FRAME_MARKER_CFG
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
import omni.usd
import omni.graph.core as og
from pxr import Usd, Sdf, UsdGeom
from isaaclab_assets import HUMANOID_MAGIC_P5_CFG
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.materials import RigidBodyMaterialCfg


from kinemic_utils import *
import json

def load_obj_info():
    obj_file_path = "scripts/tutorials/magicbot/env/obj_info.json"  # 修改为相对路径
    obj_info = {}
    with open(obj_file_path, "r") as f:
        json_lines = f.readlines()
        for line in json_lines:
            obj_info.update(json.loads(line))
    return obj_info

OBJ_INFO = load_obj_info()

joint_names = ['joint_la1', 'joint_la2', 'joint_la3', 'joint_la4', 'joint_la5', 'joint_la6', 'joint_la7', 'joint_ra1', 'joint_ra2', 'joint_ra3', 'joint_ra4', 'joint_ra5', 'joint_ra6', 'joint_ra7', None,None, None, None, None, None, None, None, None, None, None, None,  'joint_wr', 'joint_wy', 'joint_hy', 'joint_hp']

finger_names = [ 'L_thumb_proximal_yaw_joint', 'L_thumb_proximal_pitch_joint','L_thumb_intermediate_joint','L_thumb_distal_joint', 'L_index_proximal_joint','L_index_intermediate_joint',
 'L_middle_proximal_joint', 'L_middle_intermediate_joint', 'L_ring_proximal_joint', 'L_ring_intermediate_joint', 'L_pinky_proximal_joint', 'L_pinky_intermediate_joint', 'R_thumb_proximal_yaw_joint', 'R_thumb_proximal_pitch_joint', 'R_thumb_intermediate_joint', 'R_thumb_distal_joint','R_index_proximal_joint', 'R_index_intermediate_joint', 'R_middle_proximal_joint', 'R_middle_intermediate_joint', 'R_ring_proximal_joint',  'R_ring_intermediate_joint', 'R_pinky_proximal_joint','R_pinky_intermediate_joint']

joint_name_seq = ['joint_wr', 'joint_wy', 'joint_hy', 'joint_la1', 'joint_ra1', 'joint_hp', 'joint_la2', 'joint_ra2', 'joint_la3', 'joint_ra3', 'joint_la4', 'joint_ra4', 'joint_la5', 'joint_ra5', 'joint_la6', 'joint_ra6', 'joint_la7', 'joint_ra7', 'L_index_proximal_joint', 'L_middle_proximal_joint', 'L_pinky_proximal_joint', 'L_ring_proximal_joint', 'L_thumb_proximal_yaw_joint', 'R_index_proximal_joint', 'R_middle_proximal_joint', 'R_pinky_proximal_joint', 'R_ring_proximal_joint', 'R_thumb_proximal_yaw_joint', 'L_index_intermediate_joint', 'L_middle_intermediate_joint', 'L_pinky_intermediate_joint', 'L_ring_intermediate_joint', 'L_thumb_proximal_pitch_joint', 'R_index_intermediate_joint', 'R_middle_intermediate_joint', 'R_pinky_intermediate_joint', 'R_ring_intermediate_joint', 'R_thumb_proximal_pitch_joint', 'L_thumb_intermediate_joint', 'R_thumb_intermediate_joint', 'L_thumb_distal_joint', 'R_thumb_distal_joint'] 

joint_idx_seq = [2, 5, 8, 9, 10, 13, 14, 15, 18, 19, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53]

spawn_cfg = PinholeCameraCfg.from_intrinsic_matrix(
    intrinsic_matrix=[541.52, 0.0, 639.5,
                      0.0, 542.32, 479.5, 
                      0.0, 0.0, 1.0],
    width=1280,
    height=960,
    focal_length=0.202,
    focus_distance=400.0,
    f_stop=0.0,
    clipping_range=(0.01, 1e6)
)

plate_x = uniform(0.3, 0.52)
plate_y = uniform(-0.44, 0.08)

@configclass
class TableTopSceneCfg(InteractiveSceneCfg):
    """Configuration for a cart-pole scene."""

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )
    # print(f"ISAAC_NUCLEUS_DIR : {ISAAC_NUCLEUS_DIR}")

    scene = AssetBaseCfg(
        prim_path="/World/Scene",
        spawn=sim_utils.UsdFileCfg(
            usd_path="assets/Factory/Assets/ArchVis/Industrial/Buildings/Warehouse/Warehouse01.usd",
            scale=(0.01, 0.01, 0.01)
        )
    )

    table_1 = AssetBaseCfg(
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.75, 0.567, 0.045),  # 桌子位置
            rot=(0.5, 0.5, 0.5, 0.5)  # 旋转角度
        ),
        prim_path="{ENV_REGEX_NS}/Table_01",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"assets/desk/desk.usd", scale=(0.012, 0.012, 0.012)
        ),
    )
    
    table_2 = AssetBaseCfg(
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.2, -1.0, 0.045),  # 桌子位置
            rot=(0.0, 0.0, -0.70711, -0.70711)  # 旋转角度
        ),
        prim_path="{ENV_REGEX_NS}/Table_02",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"assets/desk/desk.usd", scale=(0.012, 0.012, 0.012)
        ),
    )

    zhuti = AssetBaseCfg(
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.1, -1.0, 1.4284),
            rot=(0.5, 0.5, 0.5, 0.5)
        ),
        prim_path="{ENV_REGEX_NS}/zhuti",
        spawn=sim_utils.UsdFileCfg(
            usd_path=OBJ_INFO["zhuti"]["path"], scale=OBJ_INFO["zhuti"]["scale"]
        ),
    )
    
    # daogui = AssetBaseCfg(
    #     init_state=AssetBaseCfg.InitialStateCfg(
    #         pos=(0.46882 - 0.6, 0.03314 - 0.6, 1.16572),
    #         rot=(0.70711, -0.70711, 0.0, 0.0)
    #     ),
    #     prim_path="{ENV_REGEX_NS}/daogui",
    #     spawn=sim_utils.UsdFileCfg(
    #         usd_path=OBJ_INFO["daogui"]["path"], scale=OBJ_INFO["daogui"]["scale"]
    #     ),
    # )
    
    aocao = AssetBaseCfg(
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(-0.06514, -0.68747, 1.09047),
            rot=(0.5,0.5,0.5,0.5)
        ),
        prim_path="{ENV_REGEX_NS}/aocao",
        spawn=sim_utils.UsdFileCfg(
            usd_path=OBJ_INFO["aocao"]["path"], scale=OBJ_INFO["aocao"]["scale"]
        ),
    )
    
    dianji_1 = AssetBaseCfg(
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(plate_x+0.196, plate_y+0.07, 0.993), # z = 0.993
            rot=(0.0,-0.70711,0.0,-0.70711)
        ),
        prim_path="{ENV_REGEX_NS}/dianji_1",
        spawn=sim_utils.UsdFileCfg(
            usd_path=OBJ_INFO["dianji"]["path"], scale=OBJ_INFO["dianji"]["scale"]
        ),
    )
    
    plate_a = AssetBaseCfg(
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(plate_x, plate_y, 0.033),
            rot=(0.5,0.5,0.5,0.5)
        ),
        prim_path="{ENV_REGEX_NS}/plate_a",
        spawn=sim_utils.UsdFileCfg(
            usd_path=OBJ_INFO["plate_a"]["path"], scale=OBJ_INFO["plate_a"]["scale"]
        ),
    )
    
    dianji_2 = AssetBaseCfg(
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(plate_x+0.196, plate_y+0.19, 0.993),
            rot=(0.0,-0.70711,0.0,-0.70711)
        ),
        prim_path="{ENV_REGEX_NS}/dianji_2",
        spawn=sim_utils.UsdFileCfg(
            usd_path=OBJ_INFO["dianji"]["path"], scale=OBJ_INFO["dianji"]["scale"]
        ),
    )
    
    dianji_3 = AssetBaseCfg(
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(plate_x+0.07, plate_y+0.07, 0.993),
            rot=(0.0,-0.70711,0.0,-0.70711)
        ),
        prim_path="{ENV_REGEX_NS}/dianji_3",
        spawn=sim_utils.UsdFileCfg(
            usd_path=OBJ_INFO["dianji"]["path"], scale=OBJ_INFO["dianji"]["scale"]
        ),
    )
    
    dianji_4 = AssetBaseCfg(
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(plate_x+0.07, plate_y+0.19, 0.993),
            rot=(0.0,-0.70711,0.0,-0.70711)
        ),
        prim_path="{ENV_REGEX_NS}/dianji_4",
        spawn=sim_utils.UsdFileCfg(
            usd_path=OBJ_INFO["dianji"]["path"], scale=OBJ_INFO["dianji"]["scale"]
        ),
    )
    
    dianji_5 = AssetBaseCfg(
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(plate_x+0.07, plate_y+0.3085, 0.993),
            rot=(0.0,-0.70711,0.0,-0.70711)
        ),
        prim_path="{ENV_REGEX_NS}/dianji_5",
        spawn=sim_utils.UsdFileCfg(
            usd_path=OBJ_INFO["dianji"]["path"], scale=OBJ_INFO["dianji"]["scale"]
        ),
    )
    
    dianji_6 = AssetBaseCfg(
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(plate_x+0.196, plate_y+0.3085, 0.993),
            rot=(0.0,-0.70711,0.0,-0.70711)
        ),
        prim_path="{ENV_REGEX_NS}/dianji_6",
        spawn=sim_utils.UsdFileCfg(
            usd_path=OBJ_INFO["dianji"]["path"], scale=OBJ_INFO["dianji"]["scale"]
        ),
    )

    robot = HUMANOID_MAGIC_P5_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    
    camera_head = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/p5_humanoid/link_hp/Camera_head",
        update_period=0.1,
        height=960,
        width=1280,
        data_types=["rgb", "distance_to_image_plane"],
        spawn=spawn_cfg,
        offset=CameraCfg.OffsetCfg(pos=(-0.02859, 0.00495, 0.30545), rot=(-0.21418, 0.67389, -0.67389, 0.21418), convention="ros"),
        # 旋转（需根据实际调整）[-x, w, z, -y]
    )
    
    action_graph = AssetBaseCfg(
        prim_path="/World/ActionGraph", spawn=sim_utils.UsdFileCfg(usd_path=f"assets/ActionGraph.usd"),
    )


class p5IsaacLabEnv():
    def __init__(self):
        self.init_app()
        self.init_sim()
        self.init_robot()
        # self.scene["obj"]._initialize_impl()
        self.init_memory_buffer = True
        self.get_joint_idx()
        self.stage = omni.usd.get_context().get_stage()
        self.step_count = 0

    def init_app(self):
        self.simulation_app = app_launcher.app

    def init_sim(self):
        sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device, physics_material=RigidBodyMaterialCfg(
            static_friction=50.0,
            dynamic_friction=50.0,
        )
)
        self.sim = sim_utils.SimulationContext(sim_cfg)
        # Set main camera
        self.sim.set_camera_view([3.5, 2.5, 1.5], [0.0, 0.0, 0.0])
        # Design scene
        scene_cfg = TableTopSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
        
        self.scene = InteractiveScene(scene_cfg)
        self.sim.reset()
        # Play the simulator

        self.sim_dt = self.sim.get_physics_dt()


    def init_robot(self):
        self.robot = self.scene["robot"]
        # self.robot_entity_cfg = SceneEntityCfg("robot", joint_names=[".*"], body_names=[".*"])
        self.robot_entity_cfg = SceneEntityCfg("robot", joint_names=joint_name_seq, body_names=[".*"])

        # self.robot_entity_cfg = SceneEntityCfg("robot")
        # magic_p5
        # Resolving the scene entities
        self.robot_entity_cfg.resolve(self.scene)
        

        # print(f"self.robot_entity_cfg.joint_ids : {self.robot_entity_cfg.joint_ids}\n")
        # print(f"joint names : {self.robot_entity_cfg.joint_names}")
        
        self.set_robot_position("robot", position=np.array([0.0, 0.0, 0.965]))


    def init_action_graph(self):
        camera_prim_path = "/World/envs/env_0/Robot/p5_humanoid/link_hp/Camera_head"
        node_attr_path = "/World/ActionGraph/ActionGraph/isaac_create_render_product.inputs:cameraPrim"
        attr = og.Controller.attribute(node_attr_path)
        if attr is not None:
            og.Controller.set(attr, camera_prim_path)
            print(f"✅ 成功绑定 cameraPrim: {camera_prim_path}")
        else:
            print(f"❌ 未找到节点属性: {node_attr_path}")


    def init_camera(self, path="/World/Camera"):
        if path.endswith("head"):
            # posi = (-0.11867, -0.04411, 0.2025)
            # ori = (-0.2967, 0.66134, -0.63436, 0.26868)

            posi = (-0.11867, -0.04411, 0.2025)
            ori = (-0.2967, 0.66134, -0.63436, 0.26868)
        else:
            posi = (0.02972, -0.09494, -0.14733)  # 删除多余的逗号
            ori = (0.04395, 0.0849, -0.1519, -0.98376)
        self.camera_cfg = CameraCfg(
            prim_path=path,
            height=480,
            width=640,
            spawn=PinholeCameraCfg(
                focal_length=24.0,
                horizontal_aperture=20.955,
                vertical_aperture=15.7,
                clipping_range=(0.1, 1000.0)
            ),
            offset=CameraCfg.OffsetCfg(
                pos=posi,  # 相对于机器人关节的偏移
                rot=ori,  # 旋转（需根据实际调整）[-x, w, z, -y]
                convention="ros"
            )
        )
        camera = Camera(self.camera_cfg)
        camera._initialize_impl()
        camera._is_initialized = True


    def get_joint_idx(self):
        self.joint_idx = []
        self.finger_idx = []
        for joint in joint_names:
            if joint is None:
                continue
            # self.joint_idx.append(joint_idx_seq[joint_name_seq.index(joint)])

            self.joint_idx.append(joint_name_seq.index(joint))
        for finger in finger_names:
            # self.finger_idx.append(joint_idx_seq[joint_name_seq.index(finger)])
            self.finger_idx.append(joint_name_seq.index(finger))


    def set_robot_position(self, asset_name, position):
        asset: RigidObject | Articulation = self.scene[asset_name]
        # get default root state
        env_ids = torch.arange(self.scene.num_envs, device=asset.device)
        
        root_states = asset.data.default_root_state[env_ids].clone()

        
        new_position = torch.tensor(
            [position],  # set robot position
            dtype=torch.float32,
            device="cuda" 
        )
        print(new_position)
        root_states[:, 0:3] = new_position

        positions = root_states[:, 0:3]

        # set into the physics simulation
        asset.write_root_pose_to_sim(torch.cat([positions, root_states[:, 3:7]], dim=-1))

        print(root_states[:, 0:3])

    
    def convert_p5_qpos(self, action, finger):

        action = torch.tensor(action)

        finger = torch.tensor(finger)

        self.dof = len(joint_name_seq)

        qpos = torch.zeros(1, self.dof, dtype=torch.float, device='cuda:0', requires_grad=False)

        bias = 0
        for i in range(len(action)):
            if(joint_names[i] is None):
                bias -= 1
                continue

            qpos[0][self.joint_idx[i+bias]] = action[i]

        for i in range(len(finger)):
            qpos[0][self.finger_idx[i]] = finger[i] 

        return qpos
    

    def run(self, qpos):

        # self.robot_entity_cfg._resolve_joint_names(self.scene) # huo qu suo ying

        finger_qpos = self.get_finger_qpos(qpos)
        joint_pos_des = self.convert_p5_qpos(qpos, finger_qpos)

        self.robot.set_joint_position_target(joint_pos_des, joint_ids=joint_idx_seq)
        # self.robot.set_joint_position_target(joint_pos_des)

        self.scene.write_data_to_sim()
        # perform step
        self.sim.step()

        # update sim-time
        # count += 1

        # update buffers
        self.scene.update(self.sim_dt)

        self.get_state()

        if self.init_memory_buffer:
            self.memory_buffer()
            self.init_action_graph()
            self.init_memory_buffer = False
        with torch.cuda.stream(None):
            cpu_tensor = self.scene["camera_head"].data.output["rgb"].detach().cpu()  # 异步拷贝到分页内存
            self.pinned_buffer.copy_(cpu_tensor, non_blocking=True)  # 非阻塞拷贝到固定内存
        # 同步流，确保所有传输完成
        torch.cuda.current_stream().synchronize()
        # 转换为 numpy 数组并读取
        self.image_head = self.pinned_buffer.numpy().squeeze(0)

        print(f"ISAAC_NUCLEUS_DIR : {ISAAC_NUCLEUS_DIR}")

        # print("Received shape of rgb   image: ", self.scene["camera_right"].data.output["rgb"])

    def get_finger_qpos(self, action):
        finger_qpos = np.zeros(24)
        
        finger_qpos[0] = action[14]
        finger_qpos[1] = action[15]
        for i in range(5):
            finger_qpos[i*2+2] = action[15+i]
            finger_qpos[i*2+3] = action[15+i]

        finger_qpos[12] = action[20]
        finger_qpos[13] = action[21]
        for i in range(5):
            finger_qpos[14+i*2] = action[21+i]
            finger_qpos[15+i*2] = action[21+i]
        return finger_qpos

    def get_state(self):
        # 机器人右手的路径
        self.hand_1_path = XFormPrim("/World/envs/env_0/Robot/p5_humanoid/L_index_intermediate/")  # 并非L_R_tip
        self.hand_2_path = XFormPrim("/World/envs/env_0/Robot/p5_humanoid/R_index_intermediate/")

        # 获取手部的位置
        pos1, ori1 = self.hand_1_path.get_world_poses()  # 获取 (position, orientation)
        pos2, ori2 = self.hand_2_path.get_world_poses()
        # tip_pos1, tip_ori1 = self.compute_B_pose(pos1.cpu().numpy().flatten().tolist(), ori1.cpu().numpy().flatten().tolist())
        # tip_pos2, tip_ori2 = self.compute_B_pose(pos2.cpu().numpy().flatten().tolist(), ori2.cpu().numpy().flatten().tolist())
        pos1 = pos1.cpu().numpy().flatten()
        pos2 = pos2.cpu().numpy().flatten()
        
        # print(pos1[1]-pos2[1])
        
        # # 计算目标位置（两只手的中点）
        # self.obj_pos = (tip_pos1 + tip_pos2) / 2
        # self.obj_ori = (tip_ori1 + tip_ori2) / 2
        self.obj_pos = np.array((pos1+pos2)/2)


    def compute_B_pose(self, A_xyz, A_ori):
        # 转换四元数顺序（Isaac Sim → SciPy）
        A_quat = np.array([A_ori[3], -A_ori[0], -A_ori[1], A_ori[2]])  # [w, -x, -y, z]
        R_A = R.from_quat(A_quat).as_matrix()  # 形状 (3,3)
    
        # 相对变换参数
        rel_xyz = np.array([0.0, 0.135, 0.0])  # B相对A的位移
        rel_rpy = np.array([0.0, 0.0, 0.0])    # B相对A的旋转（欧拉角）

        # 计算B的全局坐标
        B_xyz = A_xyz + R_A @ rel_xyz  # 批量矩阵乘法

        # 四元数叠加
        B_quat = np.array([A_ori[0], A_ori[1], A_ori[2], A_ori[3]]) # 转回Isaac格式，由于rel_rpy为0，旋转不变

        return B_xyz, B_quat
    

    def update_obj_pos(self, pos):
        # table_prim = XFormPrim("/World/envs/env_0/Table")
        # pos_t, ori_t = table_prim.get_world_poses()
        # print(pos_t)
        # pos_t = pos_t.squeeze(0)  # 变成 (3,)
        # # 更新 Z 轴位置
        # pos_t[2] = torch.tensor(pos[2] - 0.78, dtype=torch.float32, device=pos_t.device)  # 只改 z 轴
        # # 恢复 (1,3) 形状
        # pos_t = pos_t.unsqueeze(0)
        # table_prim.set_world_poses(pos_t, ori_t)

        # pos[0] -= 0.015
        # pos[2] += 0.06

        # self.set_robot_position("obj", position=np.array(pos))

        # hand_1_path = XFormPrim("/World/envs/env_0/Robot/p5_humanoid/L_index_intermediate/")  # 并非L_R_tip
        # hand_2_path = XFormPrim("/World/envs/env_0/Robot/p5_humanoid/L_thumb_intermediate/")
        # pos1, ori1 = hand_1_path.get_world_poses()  # 获取 (position, orientation)
        # pos2, ori2 = hand_2_path.get_world_poses()
        # self.set_robot_position("obj", position=np.array((pos1.detach().cpu().numpy()+pos2.detach().cpu().numpy())/2))
        pos = list(pos)
        orange_pos = [pos[0], pos[1], 1.0]
        bottle_pos = [pos[2], pos[3], 1.0]
        beer_pos = [pos[4], pos[5], 1.0]
        burger_pos = [pos[6], pos[7], 1.0]

        # pos = np.array([uniform(14.35, 14.55), uniform(21.85, 22.05), 1.2])
        self.set_robot_position("orange", position=np.array(orange_pos))
        self.set_robot_position("bottle", position=np.array(bottle_pos))
        self.set_robot_position("beer", position=np.array(beer_pos))
        self.set_robot_position("burger", position=np.array(burger_pos))
        # self.set_robot_position("pot", position=np.array([14.4, 22.3, 1.0]))
        # self.set_robot_position("log_box", position=np.array([14.63, 22.0, 1.0]))

        return

        print(f"----------------pos : {pos}----------------\n")

        root_state = self.scene_entities["cone"].data().default_root_state.clone()
        new_position = torch.tensor(
            [pos],  # set robot position
            dtype=torch.float32,
            device="cuda" 
        )
        root_state[:, :3] = new_position

        self.scene_entities["cone"].write_root_pose_to_sim(root_state[:, :7])
        self.scene_entities["cone"].write_root_velocity_to_sim(root_state[:, 7:])
        self.scene_entities["cone"].reset()

    def memory_buffer(self):
        source_tensor = self.scene["camera_head"].data.output["rgb"]
        buffer_shape = source_tensor.shape
        buffer_dtype = torch.uint8
        self.pinned_buffer = torch.empty(buffer_shape, dtype=buffer_dtype).pin_memory()

    def sample_point_in_circle(self, a, b, c):
        r = c * math.sqrt(random())
        theta = uniform(0, 2 * math.pi)
        x = a + r * math.cos(theta)
        y = b + r * math.sin(theta)
        return x, y

    def dh_to_htm(self, alpha, a, d, theta):
        """根据 DH 参数构造齐次变换矩阵"""
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),                np.cos(alpha),               d],
            [0,              0,                            0,                           1]
        ])

    def forward_kinematics(self, dh_params):
        """
        接受形如 [(alpha1,a1,d1,theta1), ..., (alpha_n, a_n, d_n, theta_n)] 的 DH 参数列表，
        返回齐次变换矩阵 T_0_n，表示从基座到末端。
        """
        T = np.eye(4)
        for (alpha, a, d, theta) in dh_params:
            T = T @ self.dh_to_htm(alpha, a, d, theta)
        return T[:3, 3], T[:3, :3]

    def rotm_to_rpy(self, R):
        """
        旋转变换矩阵到rpy
        """
        sy = math.sqrt(R[0,0]**2 + R[1,0]**2)
        singular = sy < 1e-6

        if not singular:
            roll  = math.atan2( R[2,1],  R[2,2])
            pitch = math.atan2(-R[2,0],  sy)
            yaw   = math.atan2( R[1,0],  R[0,0])
        else:
            roll  = math.atan2(-R[1,2], R[1,1])
            pitch = math.atan2(-R[2,0], sy)
            yaw   = 0

        return roll, pitch, yaw

#     dh = [
#         (0, 1.0, 0, np.deg2rad(30)),  # 关节1：α1=0, a1=1, d1=0, θ1=30°
#         (0, 1.0, 0, np.deg2rad(45))   # 关节2：α2=0, a2=1, d2=0, θ2=45°
#     ]
#     T = forward_kinematics(dh)
#     pos = T[:3, 3]
#     R = T[:3, :3]
#     print("末端位置 (x,y,z)：", pos)
#     print("末端旋转矩阵：\n", R)

