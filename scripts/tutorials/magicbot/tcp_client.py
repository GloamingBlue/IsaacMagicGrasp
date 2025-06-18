import socket
import struct

import sys
sys.path.append("/YOUR_PATH_TO/code/gr00t_n1/")
import numpy as np
from gr00t.eval.robot import RobotInferenceClient
TCP_HOST = "tcp://0.0.0.0"
TCP_PORT = "5555"

class getTCPData():
    def __init__(self):
        pass
        self.init_client()


    def init_client(self):
        self.policy_client = RobotInferenceClient(host=TCP_HOST, port=TCP_PORT)

        print("Available modality config available:")
        modality_configs = self.policy_client.get_modality_config()
        print(modality_configs.keys())

    def get_infer_val(self, qpos, head_image, right_hand_image):
        qpos_array = np.array(qpos)
        qpos_array = np.reshape(qpos_array,[1,30])
        obs = {
    "video.headf": head_image,
    "video.right_hand": right_hand_image,
    "state.left_arm": qpos_array[:,:7],
    "state.right_arm": qpos_array[:,7:14],
    "state.left_hand": qpos_array[:,14:20],
    "state.right_hand": qpos_array[:,20:26],
    "state.waist": qpos_array[:,26:28],
    "state.head": qpos_array[:,28:30],
    "annotation.human.action.task_description": ["do your thing!"],
        }

    #     obs = {
    # "video.headf": np.random.randint(0, 256, (1, 256, 256, 3), dtype=np.uint8),
    # "state.left_arm": np.random.rand(1, 7),
    # "state.right_arm": np.random.rand(1, 7),
    # "state.left_hand": np.random.rand(1, 6),
    # "state.right_hand": np.random.rand(1, 6),
    # "state.waist": np.random.rand(1, 3),
    # "annotation.human.action.task_description": ["do your thing!"],
    #     }
        action = self.policy_client.get_action(obs)
        return action

