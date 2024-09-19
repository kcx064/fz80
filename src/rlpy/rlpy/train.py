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
        self.timer = self.create_timer(0.02, self.callback)

        # 创建订阅者：订阅HK信息，话题类型VehicleAttitude，指定名称"hk_state"
        self.state_sub = self.create_subscription(VehicleAttitude, "hk_state", self.state_cb, 3)

        self.vehicle_cmd_pub = self.create_publisher(VehicleCmd, "hk_action", 3) 

        # 经验回放池实例化
        self.replay_buffer = ReplayBuffer(capacity=ddpg_param.buffer_size)

        # 模型实例化
        self.agent = DDPG(n_states = 2,  # 状态数
                    n_hiddens = ddpg_param.n_hiddens,  # 隐含层数
                    n_actions = 2,  # 动作数
                    action_bound = 1.0,  # 动作最大值
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
        self.state = [0, 0]  # 初始时的状态
        self.next_state = [0, 0]
        self.action = [0,0]  # 初始动作
        self.done = False  # 回合结束标记
        self.rl_times = 0
        self.actor_loss = 0

    def callback(self):
        # self.get_logger().info("hello, world!")
        # 计算reward
        target = (self.next_state[0]*100 - -0.1)**4 + (self.next_state[1]*100 - 0.56)**4
        target_old = (self.state[0]*100 - -0.1)**4 + (self.state[1]*100 - 0.56)**4
        
        if target < 1000:
            self.done = True
        else:
            self.done = False
        
        
        self.reward = (200 - 1*target)*0.001  + (target_old - target)*0.001
        # self.get_logger().info("target_old - target: %s" % (target_old - target))
        self.get_logger().info("reward: %s" % self.reward)
        self.get_logger().info("target: %s" % target)
        # self.get_logger().info("actor_loss: %s" % self.actor_loss.detach().cpu().numpy())
        # self.get_logger().info("yaw frame: %s" % (self.next_state[0]*100))
        self.get_logger().info("pitch frame: %s" % (self.next_state[1]*100))
        
        # 获取当前状态
        self.state = self.next_state
        
        # 获取当前状态对应的动作
        self.action = self.agent.take_action(self.state)
        # self.get_logger().info("action: %s" % self.action)

        # 环境更新，将action量输入真实系统，并等待返回的结果，即@self.next_state
        msg_cmd = VehicleCmd()
        msg_cmd.header.stamp = self.get_clock().now().to_msg()
        msg_cmd.header.frame_id = "cmd"
        pitch_c = self.action[0][0]*10 + self.next_state[1]*100
        yaw_c = self.action[0][1]*10 + self.next_state[0]*100
        msg_cmd.pitch_cmd = float(pitch_c)
        msg_cmd.yaw_cmd = float(yaw_c)
        self.vehicle_cmd_pub.publish(msg_cmd)
        # self.next_state, self.reward, self.done, _, _ = env.step(self.action)
        self.rl_times += 1

        # 更新经验回放池
        self.replay_buffer.add(self.state, self.action, self.reward, self.next_state, self.done)
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
            q_predict, self.actor_loss, critic_loss = self.agent.update(transition_dict)
            
            if self.rl_times % 500 == 1:
                self.agent.save_model("./test")
                self.get_logger().info("saved model!")
            
        # else:
        #     self.get_logger().info("buffer is not full")
            



    def state_cb(self, msg):
        # self.get_logger().info("state: %s" % msg)
        # self.next_state = [msg.pitch/100, msg.yaw/100, msg.frame_yaw/100, msg.frame_pitch/100, msg.md_x/100, msg.md_y/100]
        self.next_state = [msg.frame_yaw*0.01 + msg.md_x*0.01, msg.frame_pitch*0.01 + msg.md_y*0.01]




def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = NodeTrain("node_train")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy


if __name__ == '__main__':
    main()
