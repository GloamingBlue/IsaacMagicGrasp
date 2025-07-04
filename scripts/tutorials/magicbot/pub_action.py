import copy
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.node import Node


class inferRos(Node):
    def __init__(self):
        super().__init__('factoryRos')
        self.publisher_qpos = self.create_publisher(Float32MultiArray, '/act_qpos', 10)
        
    def pub_act_qpos(self, qpos):
        qpos_pub = Float32MultiArray(data=qpos)
        self.publisher_qpos.publish(qpos_pub)

class Publisher(inferRos):
    def __init__(self):
        inferRos.__init__(self)
        self.qpos = copy.deepcopy([0. for _ in range(30)])
        
        # ✅ 用你自己的动作数据替代模型推理(不要求.npy格式)，动作为位置增量
        # ✅ 动作数据分布要求
        # qpos[:7] = left_arm
        # qpos[7:14] = right_arm
        # qpos[14:20] = left_hand
        # qpos[20:26] = right_hand
        # qpos[26:28] = head
        # qpos[28:30] = waist
        self.qpos_seq = np.load("your_data/qpos_seq.npy")  # e.g. shape: (400, 30)
        
        self.qpos_idx = 0
        self.total_len = self.qpos_seq.shape[0]
        
        # 每0.1秒（100ms）触发一次，每次触发时​​自动调用节点内的self.pub_action方法​，此处可替换为其他方法
        self.timer = self.create_timer(0.1, self.pub_action)

    def pub_action(self):
        # 每次定时器触发时发布一帧
        qpos = self.qpos_seq[self.qpos_idx].tolist()
        
        self.pub_act_qpos(qpos)
        self.qpos_idx = (self.qpos_idx + 1) % self.total_len
            

if __name__ =="__main__":
    rclpy.init()
    robot_ctrl_ros = Publisher()

    # 运行节点
    rclpy.spin(robot_ctrl_ros)
    # 销毁节点，退出ROS2
    robot_ctrl_ros.destroy_node()
    rclpy.shutdown()