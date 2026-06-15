# DroneDeliverySitl

基于 ROS Noetic + PX4 + Gazebo 的四旋翼无人机自主多航点送货仿真。

## 环境要求

- Ubuntu 20.04 (WSL2)
- ROS Noetic
- PX4-Autopilot (v1.13+)
- MAVROS (`sudo apt install ros-noetic-mavros ros-noetic-mavros-extras`)

## 克隆 & 编译

```bash
cd ~/catkin_ws/src
git clone https://github.com/Orang3Whale/DroneDeliverySitl.git drone_delivery
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 运行

**终端 1** — 启动 PX4 SITL 仿真：

```bash
roslaunch px4 mavros_posix_sitl.launch
```

**终端 2** — 运行送货任务：

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch drone_delivery delivery_mission.launch
```

## 自定义航点

编辑 `config/waypoints.yaml`：

```yaml
waypoints:
  - x: 4.0
    y: 4.0
    z: 2.5
    hover_time: 5.0    # 悬停秒数，0 = 途经点

  - x: 15.0
    y: 12.0
    z: 2.5
    hover_time: 0.0    # 到达后直接降落
```

使用自定义配置文件：

```bash
roslaunch drone_delivery delivery_mission.launch waypoints_file:=my_waypoints.yaml
```

## 自定义参数

```bash
roslaunch drone_delivery delivery_mission.launch \
  takeoff_altitude:=3.0 \
  position_tolerance:=0.2 \
  waypoints_file:=three_stops.yaml
```

| 参数 | 默认值 | 说明 |
|-----|-------|------|
| `takeoff_altitude` | 2.5 | 起飞高度 (m) |
| `position_tolerance` | 0.3 | 航点到达阈值 (m) |
| `waypoints_file` | waypoints.yaml | 航点配置文件 |
| `uav_namespace` | "" | 多机命名空间前缀 |
