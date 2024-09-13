# NX每次开机需要执行一次
sudo chmod 777 /dev/ttyTHS0

# 进入项目代码路径
cd fz_80

# 编译
colcon build

# 设置环境变量
source install/setup.bash

# 运行收发节点
ros2 run fz_uart send_receive

# 运行训练节点
ros2 run rlpy train

# Tip: Ctrl+C 推出运行训练