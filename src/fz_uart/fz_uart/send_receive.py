#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from fz80_interfaces.msg import VehicleAttitude, VehicleCmd

class NodeSendReceive(Node):
    def __init__(self, name):
        super().__init__(name)  # 初始化节点名称
        self.get_logger().info("Node: %s is running!" % name)

        self.vehicle_cmd_sub = self.create_subscription(VehicleCmd, "hk_action", self.action_cb, 10)

        self.state_pub = self.create_publisher(VehicleAttitude, "hk_state", 10) 

        self.timer = self.create_timer(1.0, self.callback)

    def action_cb(self, msg):
        self.get_logger().info("action received: %s" % msg)

    def callback(self):
        self.get_logger().info("timer callback")



def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = NodeSendReceive("node_uart")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy


if __name__ == '__main__':
    main()
