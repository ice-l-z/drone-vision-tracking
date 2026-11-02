# 无人机视觉目标检索与自主追踪

把一张目标照片放进 SD 卡，四旋翼就能自己起飞、盘旋搜索、找到后飞过去悬停盯住 —— 不需要训练模型，靠 ORB 特征比对完成检索。

系统跑在 Zynq-7100 板卡（XC7Z100，PS 端双核 Cortex-A9 + PL 端 Kintex-7）上，视觉节点输出目标坐标，控制节点通过 uXRCE-DDS 驱动 PX4 飞控。

## 一次完整的作业是怎么走的

1. **读模板。** 视觉节点启动时从 SD 卡读入参考图，提取 ORB 特征点与描述子缓存，换目标只需换图片、不用改代码。
2. **找目标。** 相机以 1280×720 / 30 Hz 出帧，去畸变后提特征，与模板做 KNN 双近邻匹配，再经比值校验和单应性外点剔除。
3. **定中心。** 拿筛选后的匹配点解出单应矩阵，把模板四个角投到实时画面，用投影四边形的几何中心作目标位置 —— 比直接平均匹配点坐标稳得多，目标局部反光时不会跳。
4. **算坐标。** 中心像素经相机内参、安装外参和飞控四元数转成 NED 视线向量，与地面 Z=0 求交得到绝对坐标；同时用目标真实尺寸走一遍针孔模型算距离，两路结果互为印证。
5. **发出去。** 以 10 Hz 发布 `/object_position_ned`、`/object/bearing`、`/object/distance`。
6. **飞过去。** 控制节点 20 Hz 跑一个五态状态机：等起飞点锁存 → 解锁 → 切 Offboard 并爬到 2 m → 螺旋扩圈搜索 → 有新鲜目标就切到追踪悬停。

## 目录结构

```
.
├── 无人机识别定位追踪/
│   ├── ORB相机侦测.py      # 视觉节点：模板匹配、单应定位、测距与坐标解算
│   └── 多旋翼搜圈.py        # 控制节点：Offboard 状态机 + 螺旋搜索轨迹
├── ros2_ws/src/
│   ├── px4_msgs/           # PX4 消息定义
│   ├── px4_ros_com/        # PX4 ↔ ROS 2 桥接示例
│   └── gazebo_model_attachment_plugin-humble/
└── README.md
```

两个业务脚本是独立节点，不参与 colcon 编译，装好依赖直接 `python3` 跑。

## 依赖

- Ubuntu 22.04 + ROS 2 Humble
- Python 3.10，`rclpy`、`opencv-python`、`numpy`
- PX4 v1.17.0（源码不在本仓库，去 [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) 取对应 tag）
- 机载端开启 uXRCE-DDS

## 编译与运行

```bash
cd ros2_ws
colcon build --packages-select px4_msgs px4_ros_com
source install/setup.bash

python3 无人机识别定位追踪/ORB相机侦测.py    # 终端 1
python3 无人机识别定位追踪/多旋翼搜圈.py      # 终端 2
```

跑之前有两处要改成实际值：`TEMPLATE_PATH`（模板图路径，代码里默认写的是 `iamge.jpg`）、以及相机内参 `FX/FY/CX/CY` 和安装外参 `R_B_C`（按自己的标定结果填）。

## 话题

| 方向 | 话题 | 类型 |
|---|---|---|
| 发 | `/object_position_ned` | `geometry_msgs/PointStamped` |
| 发 | `/object/bearing` | `std_msgs/Float32` |
| 发 | `/object/distance` | `std_msgs/Float32` |
| 发 | `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` |
| 发 | `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` |
| 发 | `/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` |
| 收 | `/fmu/out/vehicle_local_position_v1` | `px4_msgs/VehicleLocalPosition` |
| 收 | `/fmu/out/vehicle_status` | `px4_msgs/VehicleStatus` |

## 可调参数

视觉侧（`ORB相机侦测.py`）：

| 参数 | 值 | 作用 |
|---|---|---|
| `ORB_NFEATURES` | 1500 | 单帧特征点上限 |
| `RATIO_THRESH` | 0.75 | 比值校验阈值 |
| `MIN_MATCH_COUNT` | 20 | 解单应矩阵所需最少内点 |
| `RANSAC_THRESH` | 5.0 px | 重投影误差阈值 |
| `PUBLISH_RATE_HZ` | 10 | 目标信息发布频率 |
| `TARGET_STALE_TIMEOUT` | 1.0 s | 多久没新数据就停发 |

控制侧（`多旋翼搜圈.py`）：

| 参数 | 值 | 作用 |
|---|---|---|
| `TAKEOFF_ALTITUDE` | 2.0 m | 定高 |
| `START_RADIUS` | 3.0 m | 搜索起始半径 |
| `RADIUS_STEP` | 1.0 m | 每绕一圈往外扩多少 |
| `ANGULAR_VEL` | 0.4 rad/s | 绕圈角速度（一圈约 15.7 s） |
| `HEIGHT_THRESHOLD` | 0.3 m | 判定"到高度了"的容差 |
| `TARGET_FRESH_TIMEOUT` | 1.0 s | 目标数据保鲜窗口 |
| `LOOP_HZ` | 20 Hz | 控制频率 |

## 几个实现上的取舍

**匹配宁可少认，不可认错。** 三级筛选后内点不足 20 个就直接放弃这一帧、沿用上一次结果，而不是硬算一个可能错的位置。地面纹理和草地反光是最容易骗过 ORB 的两类干扰，这一步能压住。

**中心点取自投影四边形，不取匹配点均值。** 匹配点的分布随纹理密度变化，均值会被密集区域拉偏；单应矩阵把模板四角整体投过去再取中心，目标被遮挡一角时也能靠其余三点撑住。

**两路测距互相兜底。** 机身倾斜时纯靠像素宽度算距离会偏，所以另一路用姿态矩阵把视线投到地面求交；两路都拿不到才认为本帧无效。

**丢目标就退回搜索，不硬追。** 视觉端 1 s 没更新就停发，控制端 1 s 没收到就退出追踪、回到绕圈。短暂遮挡靠 10 Hz 的持续发布平滑过去，真丢了也不会悬在原地等。

**目标坐标有两条来源。** 优先用 `/object_position_ned` 的绝对坐标；这条断了就用"当前航向 + 相对方位角 + 水平距离"现算，避免单个话题抖动导致追踪中断。

## 实测记录

测试条件：定高 2 m，平坦地面，Z=0 约束。

| 项目 | 结果 |
|---|---|
| 模板特征点 | 850 ～ 1500 个，SD 卡读取 < 50 ms |
| 匹配帧率 | 30 FPS @ 1280×720 |
| 旋转适应 | 0°～360° 均能检出，内点 30 ～ 160 个 |
| 三维距离误差 | ≤ 0.08 m（卷尺与激光测距仪对照） |
| 悬停水平误差 | ≤ 0.35 m |

## 第三方组件

`PX4-Autopilot`、`px4_msgs`、`px4_ros_com` 均为 BSD-3-Clause，版权归 PX4 Development Team；`gazebo_model_attachment_plugin-humble` 见其目录内 LICENSE。`无人机识别定位追踪/` 下的两个节点为本项目自研代码。

## 还能往哪走

- 把去畸变、滤波和角点检测搬到 PL 端做硬件流水线，目标 60 FPS 以上
- 接二轴云台 + 卡尔曼滤波，追地面移动目标
- 用由大到小的多级模板做降落导引，往厘米级靠
