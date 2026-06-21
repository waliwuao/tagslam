# TagSLAM — PID 控制小车定位系统

基于 AprilTag 摄像头的 PID 闭环控制，将搭载摄像头的小车移动到指定目标位置。

## 环境要求

- ROS2 Humble / Jazzy / Rolling
- GTSAM、OpenCV、Eigen3、Boost、yaml-cpp
- USB 摄像头 + AprilTag 标签 (默认 3 个，ID 0/1/2，尺寸 0.12m)

```bash
# 安装系统依赖
sudo apt install ros-${ROS_DISTRO}-usb-cam \
                 ros-${ROS_DISTRO}-apriltag-detector \
                 ros-${ROS_DISTRO}-flex-sync
```

## 构建

```bash
colcon build --symlink-install
```

## 配置

所有配置在 `config/` 目录：

| 文件 | 用途 |
|---|---|
| `cameras.yaml` | 摄像头内参、图像话题、标签检测器选择 |
| `tagslam.yaml` | AprilTag 位姿定义、SLAM 优化参数 |
| `camera_poses.yaml` | 摄像头初始位姿先验 (未知时用弱先验) |
| `pid_control.yaml` | PID 参数、目标点、超时/丢Tag限制条件 |

### pid_control.yaml 关键参数

```yaml
target:
  x: 1.0          # 目标 X [m]
  y: 0.0          # 目标 Y [m]
  angular: 3.14159 # 目标朝向 [rad], π = 面朝 Tag 0

max_duration: 3.0          # 单次调节最大时长 [秒]
max_missing_frames: 10     # 连续丢 Tag 帧数上限 (超限停止)
done_hold_frames: 5        # 死区内保持 N 帧视为到达
dead_zone_linear: 0.02     # 线位移死区 [m]
dead_zone_angular: 0.05    # 角位移死区 [rad]
```

## 运行

### 终端 1：启动全部节点

```bash
./start.sh
```

按顺序启动：`usb_cam → sync_and_detect → tagslam → pid_control`

### 终端 2：触发一次 PID 调节

```bash
./trigger.sh
```

每次运行触发一次 PID 调节，将小车移动到 `pid_control.yaml` 中配置的目标点。

## 工作流程

```
./start.sh
  ├─ usb_cam          → 发布 /camera/image_raw
  ├─ sync_and_detect  → 检测 AprilTag, 发布 /detector/tags
  ├─ tagslam          → SLAM 估计位姿, 发布 TF: world→cam0
  └─ pid_control      → 等待服务触发
                          ↑
./trigger.sh ─────────────┘
                          ↓
                   PID 调节 (20Hz)
                   ├─ 到达目标 → 停止, 发布零速
                   ├─ >max_duration → 停止
                   └─ >max_missing → 停止
                   PID 停止后回到等待, 可再次触发
```

## 包结构

| 包 | 类型 | 说明 |
|---|---|---|
| `tagslam` | C++ | 主 SLAM + 标签同步检测 |
| `pid_control` | Python | PID 控制节点 (服务触发, 无UI) |
| `apriltag_detector` | C++ | 标签检测框架 (umich/mit 插件) |
| `apriltag_mit` | C | MIT AprilTag 底层库 |
| `apriltag_msgs` | 消息定义 | AprilTagDetection 等 |
| `flex_sync` | C++ 头文件 | 动态传感器同步 |
