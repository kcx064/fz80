#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from fz80_interfaces.msg import VehicleAttitude, VehicleCmd

from .submodules.test import getdata
import time
import numpy as np

# # 配置串口参数
port = '/dev/ttyTHS0'  # 替换为你的串口号
baudrate = 115200  # 波特率
timeout = 1  # 读取超时设置（秒）

class NodeSendReceive(Node):
    def __init__(self, name):
        super().__init__(name)  # 初始化节点名称
        self.get_logger().info("Node: %s is running!" % name)

        self.vehicle_cmd_sub = self.create_subscription(VehicleCmd, "hk_action", self.action_cb, 10)

        self.state_pub = self.create_publisher(VehicleAttitude, "hk_state", 10) 

        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.Getdata=getdata(port,baudrate)
        self.Getdata.initialize()

    def action_cb(self, msg):
        self.get_logger().info("action received: pitch cmd: %s yaw cmd: %s" % (msg.pitch_cmd, msg.yaw_cmd))
        
        self.rollCmd = 0
        self.pitchCmd = float(msg.pitch_cmd)
        self.yawCmd = float(msg.yaw_cmd)
        self.sendData = np.array((self.yawCmd, self.pitchCmd, self.rollCmd))
        
        self.Getdata.send(self.sendData) #发送

    def timer_callback(self):
        # self.get_logger().info("timer callback")
        par_data = self.Getdata.run() # 接收
        # for key, value in par_data.items():
        #     print(f"{key}: {value}", end=' ')
        #     # print(par_data["sync1"])
        #     print()  # 换行
        
        FK_pitch = par_data["FK_pitch"]
        FK_yaw = par_data["FK_yaw"]
        Frame_Angle_Pitch = par_data["Frame_Angle_Pitch"]
        Frame_Angle_Yaw = par_data["Frame_Angle_Azimuth"]
        OffTarget_x = par_data["OffTarget_x"]
        OffTarget_y = par_data["OffTarget_y"]
        
        msg_state = VehicleAttitude()
        msg_state.header.stamp = self.get_clock().now().to_msg()
        msg_state.header.frame_id = "send_receive"
        msg_state.pitch = float(FK_pitch)
        msg_state.yaw = float(FK_yaw)
        msg_state.frame_pitch = float(Frame_Angle_Pitch)
        msg_state.frame_yaw = float(Frame_Angle_Yaw)
        msg_state.md_x = float(OffTarget_x)
        msg_state.md_x = float(OffTarget_y)

        self.state_pub.publish(msg_state)




def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = NodeSendReceive("node_uart")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy


if __name__ == '__main__':
    main()
