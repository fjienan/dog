
编译代码（不保证能跑）：

```
（~/.bashrc 参考）
#source /usr/share/gazebo-11/setup.sh
#source /usr/share/gazebo/setup.bash
# source /opt/ros/humble/setup.bash
source ~/gazebo_simulation/install/setup.bash
export GAZEBO_MODEL_PATH=$HOME/gazebo_simulation/src/my_robot_description/models:/usr/share/gazebo-11/models
export GAZEBO_RENDER_ENGINE=ogre

加载世界：
cd ~/gazebo_simulation
rm -rf build/ install/
colcon build --symlink-install
source ~/.bashrc
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml

使用控制器：
ros2 launch myrobot_controller controller.launch.py
可以用这玩意检查是否成功启动：
ros2 control list_controllers
输入参数：
ros2 topic pub /simple_velocity_controller/commands std_msgs/msg/Float64MultiArray "layout:
  dim: []
  data_offset: 0
data: [0, 0, 0, 20, 20, 20]"

```

qwq，现在的启动程序（驱动还没改好驱动还没改好驱动还没改好驱动还没改好）：

```
colcon build
source ~/.bashrc
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml

ros2 launch myrobot_controller controller.launch.py

ros2 topic pub /myrobot_controller/cmd_vel geometry_msgs/msg/TwistStamped "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
twist:
  linear:
    x: 10.0
    y: 10.0
    z: 10.0
  angular:
    x: 2.0
    y: 2.0
    z: 2.0"

```

没写luanch文件前启动节点及其相应参数文件

``` terminal
ros2 run omni_mpc_controller MPC --ros-args --params-file $(ros2 pkg prefix omni_mpc_controller)/share/omni_mpc_controller/config/params.yaml
```

输出报错是正常的，因为没有收到雷达数据，所以对应的接收到一个空数组，因此报错，至于有没有其他问题尚不得而知，目前确定的是参数可以正常导入到文件当中