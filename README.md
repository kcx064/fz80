# HIL运行

## NX每次开机需要执行一次
sudo chmod 777 /dev/ttyTHS0

## 确认：已经进入项目代码路径
cd fz_80

## 编译（不修改代码不需要编译）
colcon build

## 设置环境变量
source install/setup.bash

## 运行收发节点
ros2 run fz_uart send_receive

## 运行训练节点
ros2 run rlpy train

## Tip: Ctrl+C 退出运行训练

# SIM运行

1.编译（不修改代码不需要编译）
colcon build

2.设置环境变量
source install/setup.bash

3. 运行仿真节点
ros2 run hk_sim sim

4. 运行训练
新开一终端：ros2 run rlpy train