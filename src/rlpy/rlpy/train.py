#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class NodeTtrain(Node):
    def __init__(self, name):
        super().__init__(name)  # 初始化节点名称
        self.get_logger().info("大家好，我是%s!" % name)
        self.timer = self.create_timer(1.0, self.callback)

    def callback(self):
        self.get_logger().info("hello, world!")



def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = NodeTtrain("node_train")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy


if __name__ == '__main__':
    main()
