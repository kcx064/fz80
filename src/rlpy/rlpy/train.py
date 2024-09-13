#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import torch

from .submodules.parsers import ddpg_param
from .submodules.DDPGmodel import ReplayBuffer, DDPG

from fz80_interfaces.msg import VehicleAttitude, VehicleCmd


class NodeTrain(Node):
    def __init__(self, name):
        super().__init__(name)  # 初始化节点名称
        self.get_logger().info("Node: %s is running!" % name)

        self.device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
        self.timer = self.create_timer(1.0, self.callback)

        # 创建订阅者：订阅HK信息，话题类型VehicleAttitude，指定名称"hk_state"
        self.state_sub = self.create_subscription(VehicleAttitude, "hk_state", self.state_cb, 10)

        self.vehicle_cmd_pub = self.create_publisher(VehicleCmd, "hk_action", 10) 

        # 经验回放池实例化
        self.replay_buffer = ReplayBuffer(capacity=ddpg_param.buffer_size)

        # 模型实例化
        self.agent = DDPG(n_states = 6,  # 状态数
                    n_hiddens = ddpg_param.n_hiddens,  # 隐含层数
                    n_actions = 2,  # 动作数
                    action_bound = 20.0,  # 动作最大值
                    sigma = ddpg_param.sigma,  # 高斯噪声
                    actor_lr = ddpg_param.actor_lr,  # 策略网络学习率
                    critic_lr = ddpg_param.critic_lr,  # 价值网络学习率
                    tau = ddpg_param.tau,  # 软更新系数
                    gamma = ddpg_param.gamma,  # 折扣因子
                    device = self.device
                    )
        
        self.return_list = []  # 记录每个回合的return
        self.mean_return_list = []  # 记录每个回合的return均值

        self.episode_return = 0 # 累计每条链上的reward
        self.state = [0, 0, 0, 0, 0, 0]  # 初始时的状态
        self.next_state = [0, 0, 0, 0, 0, 0]
        self.action = [0,0]  # 初始动作
        self.done = False  # 回合结束标记
        self.rl_times = 0

    def callback(self):
        # self.get_logger().info("hello, world!")
        # TODO: 通过话题获取当前状态
        
        # 获取当前状态对应的动作
        self.action = self.agent.take_action(self.state)
        self.get_logger().info("action: %s" % self.action)

        # TODO：环境更新，将action量输入真实系统，并等待返回的结果，即@self.next_state
        msg_cmd = VehicleCmd()
        msg_cmd.header.stamp = self.get_clock().now().to_msg()
        msg_cmd.header.frame_id = "cmd"
        pitch_c = self.action[0][0]
        yaw_c = self.action[0][1]
        msg_cmd.pitch_cmd = float(pitch_c)
        msg_cmd.yaw_cmd = float(yaw_c)
        self.vehicle_cmd_pub.publish(msg_cmd)
        # self.next_state, self.reward, self.done, _, _ = env.step(self.action)
        self.rl_times += 1

        # 计算reward
        self.reward = 1000 - self.next_state[4]*self.next_state[4] - self.next_state[5]*self.next_state[5]
        print("reward ",self.reward)

        # 更新经验回放池
        self.replay_buffer.add(self.state, self.action, self.reward, self.next_state, self.done)
        # 状态更新
        self.state = self.next_state
        # 累计每一步的 reward
        self.episode_return += self.reward
 
        # 如果经验池超过容量，开始训练
        if self.replay_buffer.size() > ddpg_param.min_size:
            # 经验池随机采样batch_size组
            s, a, r, ns, d = self.replay_buffer.sample(ddpg_param.batch_size)
            # 构造数据集
            transition_dict = {
                'states': s,
                'actions': a,
                'rewards': r,
                'next_states': ns,
                'dones': d,
            }
            # 模型训练
            self.agent.update(transition_dict)

        if self.rl_times == 10:
            self.agent.save_model("./test")

    def state_cb(self, msg):
        # self.get_logger().info("state: %s" % msg)
        self.next_state = [msg.pitch, msg.yaw, msg.frame_pitch, msg.frame_yaw, msg.md_x, msg.md_y]




def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = NodeTrain("node_train")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy


if __name__ == '__main__':
    main()
