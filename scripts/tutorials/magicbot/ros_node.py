import copy
import sys

import numpy as np
import time

from rclpy.node import Node
from std_msgs.msg import String, Int32
import cv2

from std_msgs import msg

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import message_filters
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, CompressedImage
# from finger_unit import fingerUnit

PI = 3.141592654


class IsaacLabRos(Node):
    def __init__(self):
        super().__init__('IsaacLabRos')

        self.subscription_act_qpos = self.create_subscription(Float32MultiArray, '/act_qpos', self.act_qpos_callback, 10)

        self.subscription_finger_qpos = self.create_subscription(Float32MultiArray, '/finger_qpos', self.finger_qpos_callback, 10)

        self.subscription_foot_qpos = self.create_subscription(Float32MultiArray, '/foot_qpos', self.foot_qpos_callback, 10)
        
        self.timer_period_render = self.create_timer(0.005, self.period_render)
        self.subscription_save_data = self.create_subscription(Int32, '/start_save_data', self.start_save_callback, 10)

        self.publisher_obj_pos = self.create_publisher(Float32MultiArray, '/obj_pos', 10)
        self.publisher_act_qpos = self.create_publisher(Float32MultiArray, '/true_act_qpos', 10)
        self.publisher_finger_qpos = self.create_publisher(Float32MultiArray, '/true_finger_qpos', 10)
        self.publisher_foot_qpos = self.create_publisher(Float32MultiArray, '/true_foot_qpos', 10)

        self.subscribe_obj_pos =  self.create_subscription(Float32MultiArray, '/obj_pos_data', self.obj_pos_callback, 10)
        
        self.subscribe_pos_mark =  self.create_subscription(Float32MultiArray, '/pos_mark', self.pos_mark_callback, 10)

        self.init_qpos_val()
        self.init_image_pub()

        # self.finger_m = fingerUnit()

    def init_qpos_val(self):
        self.qpos = np.zeros(30)
        self.foot_qpos = np.zeros(12)
        self.finger_qpos = np.zeros(24)

    def pub_obj_pos(self,pos):
        pos_pub = Float32MultiArray(data=pos)
        self.publisher_obj_pos.publish(pos_pub)
        
    def pub_act_qpos(self,pos):
        pos_pub = Float32MultiArray(data=pos)
        self.publisher_act_qpos.publish(pos_pub)

    def pub_finger_qpos(self,pos):
        pos_pub = Float32MultiArray(data=pos)
        self.publisher_finger_qpos.publish(pos_pub)
    
    def pub_foot_qpos(self,pos):
        pos_pub = Float32MultiArray(data=pos)
        self.publisher_foot_qpos.publish(pos_pub)
    
    def init_image_pub(self):
        self.bridge_head = CvBridge()
        # self.bridge_left_hand = CvBridge()
        # self.bridge_right_hand = CvBridge()
        self.publisher_head_ = self.create_publisher(Image, '/rgbd3/color/image_raw', 10)
        # self.publisher_left_hand = self.create_publisher(Image, '/rgbd1/color/image_rect_raw', 10)
        # self.publisher_right_hand = self.create_publisher(Image, '/rgbd2/color/image_rect_raw', 10)

    def act_qpos_callback(self, msg):
        # self.get_logger().info('GET : "%s"' % msg.data)
        data = msg.data
        self.qpos = data
        print(f"get qpos : {self.qpos}")

    def foot_qpos_callback(self, msg):
        # self.get_logger().info('GET : "%s"' % msg.data)
        data = msg.data
        self.foot_qpos = data


    def finger_qpos_callback(self, msg):
        # self.get_logger().info('GET : "%s"' % msg.data)
        data = msg.data
        self.finger_qpos = self.finger_m.get_finger_qpos(data)


    def period_render(self):
        pass

    def start_save_callback(self,msg):
        pass

    def pub_image(self, head_image):  #, left_hand_image, right_hand_image):
        data_head = self.bridge_head.cv2_to_imgmsg(head_image, encoding="bgr8") 
        # data_left_hand = self.bridge_head.cv2_to_imgmsg(left_hand_image,encoding="bgr8") 
        # data_right_hand = self.bridge_head.cv2_to_imgmsg(right_hand_image,encoding="bgr8") 

        self.publisher_head_.publish(data_head)
        # self.publisher_left_hand.publish(data_left_hand)
        # self.publisher_right_hand.publish(data_right_hand)

    def pos_mark_callback(self, data):
        pass

    def obj_pos_callback(self, data):
        pass


# if __name__ == '__main__':
#     main()
