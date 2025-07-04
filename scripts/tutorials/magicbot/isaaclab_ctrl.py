import rclpy
import numpy as np
from ros_node import IsaacLabRos
from env_factory import p5IsaacLabEnv
from cv_bridge import CvBridge, CvBridgeError


PI = 3.1415926

class issaclabCtrl(IsaacLabRos):
    def __init__(self) -> None:
        IsaacLabRos.__init__(self)
        self.sim = p5IsaacLabEnv()
        
        

    def period_render(self):
        self.init_qpos()

        self.sim.run(self.qpos)

        self.pub_obj_pos(self.sim.obj_pos)
        self.pub_act_qpos(self.qpos)
        self.pub_finger_qpos(self.finger_qpos)
        self.pub_foot_qpos(self.foot_qpos)

        # self.sim.step_count += 1
        # if(self.sim.step_count<50):
        #     self.qpos = np.zeros(30)
        #     self.qpos[1] =0.5*PI
        #     self.qpos[8] = -0.5*PI

        # elif(self.sim.step_count<100):
        #     self.qpos = np.zeros(30)
        #     self.qpos[2] = 0.5* PI
        #     self.qpos[3] = -0.5* PI
        #     self.qpos[4] = -PI
        #     self.qpos[9] = -0.5* PI
        #     self.qpos[10] = 0.5* PI
        #     self.qpos[11] = PI


        # self.sim.step(self.qpos, self.foot_qpos)

        # print(self.sim.scene["camera_right"].data.output["rgb"].cpu().numpy().shape)

        # self.pub_image(self.sim.scene["camera_head"].data.output["rgb"].detach().cpu().numpy()[0])
        self.pub_image(self.sim.image_head)
        # self.pub_obj_pos(self.sim.obj_pos)

    # def start_save_callback(self, msg):
    #     pos_quat = (0.45, -0.14, 1.1, -0.7071082, 1.0383573e-08, 0.7071055, 1.9900469e-08)

    #     self.sim.update_obj_pos(pos_quat)

    def init_qpos(self):
        self.sim.step_count += 1
        if(self.sim.step_count<50):
            self.qpos = np.zeros(30)
            self.qpos[1] =0.5*PI
            self.qpos[8] = -0.5*PI

        elif(self.sim.step_count<100):
            self.qpos = np.zeros(30)
            self.qpos[:14] = [
                -0.5, 0.2,  1.7,-1.2, -3.14, 0.0, -0.0, 
                -0.5, -0.2, -1.7, 1.2, 3.14, 0.0, -2.0, ]
                # 3.07, 3.08, 3.08, 3.08, 0.92, 2.81, 
                # 3.07, 3.08, 3.08, 3.08, 0.92, 2.83, 
                # 0.01, -0.01, 0.0, 0.0]
            
            # self.qpos[2] = 0.5* PI
            # self.qpos[3] = -0.5* PI
            # self.qpos[4] = -PI
            # self.qpos[9] = -0.5* PI
            # self.qpos[10] = 0.5* PI
            # self.qpos[11] = PI


    def obj_pos_callback(self, msg):
        pos = msg.data
        self.sim.update_obj_pos(pos)



def main():
    rclpy.init()
    isaac_manage = issaclabCtrl()
    # 运行节点
    rclpy.spin(isaac_manage)
    #
    # # 销毁节点，退出ROS2
    isaac_manage.destroy_node()
    rclpy.shutdown()


if __name__ =="__main__":
    main()
