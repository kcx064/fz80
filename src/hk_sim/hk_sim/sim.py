#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from fz80_interfaces.msg import VehicleAttitude, VehicleCmd
import numpy as np
import random

class NodeSim(Node):
    def __init__(self, name):
        super().__init__(name)  # 初始化节点名称
        # 生成随机目标
        self.pitch_t = 20*random.uniform(-1, 1)
        self.yaw_t = 20*random.uniform(-1, 1)
        
        # 假设初始控制量为0
        self.pitch_cmd = 0
        self.yaw_cmd = 0

        # 假设导引初始对准目标
        self.pitch_frame = self.pitch_t
        self.yaw_frame = self.yaw_t

        self.get_logger().info("Node: %s is running!" % name)
        self.state_pub = self.create_publisher(VehicleAttitude, "hk_state", 10) 
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.vehicle_cmd_sub = self.create_subscription(VehicleCmd, "hk_action", self.action_cb, 10)

    def reset(self):
        self.pitch_t = 20*random.uniform(-1, 1)
        self.yaw_t = 20*random.uniform(-1, 1)
        ## 增量仿真模式
        # self.pitch_frame = self.pitch_t
        # self.yaw_frame = self.yaw_t
        
        ## 绝对量控制模式
        self.pitch_frame = self.pitch_t - self.pitch_cmd
        self.yaw_frame = self.yaw_t - self.yaw_cmd

    def action_cb(self, msg):
        self.get_logger().info("pitch fr:%s yaw fr:%s pitch tg:%s yaw tg:%s" % (self.pitch_frame, self.yaw_frame, self.pitch_t, self.yaw_t))
        
        ## 增量仿真模式
        # self.pitch_frame -=  msg.pitch_cmd
        # self.yaw_frame -= msg.yaw_cmd

        ## 绝对量控制模式
        self.pitch_cmd = msg.pitch_cmd
        self.yaw_cmd = msg.yaw_cmd
        self.pitch_frame = self.pitch_t - self.pitch_cmd
        self.yaw_frame = self.yaw_t - self.yaw_cmd

        if abs(self.pitch_frame) < 0.1 and abs(self.yaw_frame) < 0.1:
            self.reset()


    def timer_callback(self):        
        msg_state = VehicleAttitude()
        msg_state.header.stamp = self.get_clock().now().to_msg()
        msg_state.header.frame_id = "hk_sim"
        msg_state.pitch = 0.0
        msg_state.yaw = 0.0
        msg_state.frame_pitch = self.pitch_frame
        msg_state.frame_yaw = self.yaw_frame
        msg_state.md_x = 0.0
        msg_state.md_x = 0.0

        self.state_pub.publish(msg_state)




def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = NodeSim("node_sim")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy


if __name__ == '__main__':
    main()

