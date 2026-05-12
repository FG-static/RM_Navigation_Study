# PB_RM_Simulation 学习笔记

> 深圳北理莫斯科大学北极熊战队 RoboMaster 哨兵导航仿真/实车包
> 项目地址：https://github.com/LihanChen2004/pb_rm_simulation
> 技术栈：ROS 2 Humble + Gazebo Classic 11 + Navigation2 + FAST_LIO/Point_LIO

---

## 1. 项目简介

全向移动小车（麦克纳姆轮）+ Livox Mid-360 雷达 + IMU，在 RMUC（7V7）/ RMUL（3V3）地图中进行导航仿真。支持边建图边导航（mapping）和已知地图导航（nav）两种模式，仿真参数可直接移植到实车。

| 特性 | 说明 |
|------|------|
| 底盘 | 4 轮麦克纳姆轮，全向移动 |
| 雷达 | Livox Mid-360（非重复扫描式 3D 激光雷达） |
| IMU | Gazebo 仿真 IMU 插件 |
| 仿真器 | Gazebo Classic 11 |
| 导航 | Nav2 + TEB 局部规划器 |
| 建图 | FAST_LIO / Point_LIO |

---

## 2. 环境要求

| 项目 | 要求 |
|------|------|
| 系统 | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| 仿真器 | Gazebo Classic 11 |
| 其他 | Livox SDK2, Docker（推荐） |

> **注意**：项目基于 Gazebo Classic 11，在 Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic 下无法直接编译（Gazebo Classic API 已移除）。建议使用 Docker 容器运行。

---

## 3. 搭建流程

### 3.1 安装 Docker

```bash
sudo apt remove -y docker docker-engine docker.io containerd runc 2>/dev/null
sudo apt update && sudo apt install -y ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update && sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo usermod -aG docker $USER && newgrp docker
```

Docker Hub 拉取超时则配置国内镜像：

```bash
sudo tee /etc/docker/daemon.json <<-'EOF'
{
  "registry-mirrors": ["https://docker.1ms.run", "https://docker.xuanyuan.me"]
}
EOF
sudo systemctl daemon-reload && sudo systemctl restart docker
```

### 3.2 克隆项目

```bash
cd ~
git clone --recursive https://github.com/LihanChen2004/pb_rm_simulation.git
cd pb_rm_simulation
```

> **注意**：`costmap_converter` 子模块的原作者 fork 可能已删除，需修改 `.gitmodules` 中 URL 为 `https://github.com/rst-tu-dortmund/costmap_converter.git`。

### 3.3 Dockerfile

原 Dockerfile 有问题（鱼香ROS 换源失败、sudo 多余、pip 限制），重写如下：

```dockerfile
FROM ros:humble-ros-base

RUN mkdir -p /ros_ws
WORKDIR /ros_ws/

RUN apt-get update && apt-get install -y \
    wget python3-pip python3-rosdep \
    clang clangd clang-format cmake git zsh \
    libgoogle-glog-dev libeigen3-dev libsuitesparse-dev \
    libg2o-dev libg2o-types-slam2d libg2o-types-slam3d \
    libg2o-stuff-sampler libg2o-solver-cholmod libg2o-solver-eigen libg2o-solver-csparse \
    ros-humble-nav2-dwb-controller \
    && rosdep init 2>/dev/null || true \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install

RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.2.0/zsh-in-docker.sh)" -- \
    -t jispwoso -p git \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-syntax-highlighting && \
    chsh -s /bin/zsh

RUN echo 'export TERM=xterm-256color\n\
source /opt/ros/humble/setup.zsh\n\
eval "$(register-python-argcomplete3 ros2)"\n\
eval "$(register-python-argcomplete3 colcon)"\n'\
>> /root/.zshrc

RUN sed --in-place --expression '$isource "/opt/ros/humble/setup.sh"' /ros_entrypoint.sh
```

### 3.4 构建镜像

```bash
docker build -t pb_rm_simulation .
```

### 3.5 启动容器

```bash
xhost +local:docker
docker run -it --rm --privileged \
  -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --network=host \
  -v ~/pb_rm_simulation:/ros_ws/pb_rm_simulation \
  pb_rm_simulation
```

### 3.6 容器内编译

```bash
cd /ros_ws/pb_rm_simulation
git config --global --add safe.directory '*'
git submodule update --init --recursive
rosdep install -r --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

---

## 4. 运行仿真

### 4.1 边建图边导航

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUL mode:=mapping lio:=fastlio lio_rviz:=False nav_rviz:=True
```

### 4.2 已知地图导航

```bash
ros2 launch rm_nav_bringup bringup_sim.launch.py \
  world:=RMUL mode:=nav lio:=fastlio localization:=slam_toolbox lio_rviz:=False nav_rviz:=True
```

### 4.3 参数说明

| 参数 | 可选值 | 说明 |
|------|--------|------|
| `world` | `RMUL` / `RMUC` | 仿真地图（3V3 / 7V7） |
| `mode` | `mapping` / `nav` | 边建图边导航 / 已知地图导航 |
| `lio` | `fastlio` / `pointlio` | LIO 算法（Point_LIO 频率更高但 CPU 占用更大） |
| `localization` | `slam_toolbox` / `amcl` / `icp` | 定位算法（仅 `nav` 模式） |
| `lio_rviz` | `True` / `False` | 显示 LIO 点云可视化 |
| `nav_rviz` | `True` / `False` | 显示 Navigation2 可视化 |

### 4.4 键盘控制

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 5. 包结构与功能

### 5.1 模块划分

```
pb_rm_simulation/src/
├── rm_driver/                          # 驱动层
│   └── livox_ros_driver2               # Livox 雷达驱动（子模块）
├── rm_localization/                    # 定位层
│   ├── fast_lio                        # FAST_LIO 建图（子模块）
│   ├── point_lio                       # Point_LIO 建图（子模块）
│   └── icp_registration                # ICP 点云配准定位
├── rm_navigation/                      # 导航层
│   ├── rm_navigation                   # Nav2 参数 + launch
│   ├── fake_vel_transform              # TF 坐标变换
│   ├── teb_local_planner/              # TEB 局部规划器（子模块）
│   │   ├── teb_local_planner
│   │   └── teb_msgs
│   └── costmap_converter/              # 代价地图转换（子模块）
│       ├── costmap_converter
│       └── costmap_converter_msgs
├── rm_nav_bringup/                     # 顶层启动
├── rm_perception/                      # 感知层
│   ├── imu_complementary_filter        # IMU 互补滤波
│   ├── linefit_ground_segementation_ros2/
│   │   ├── linefit_ground_segmentation  # 地面分割算法
│   │   └── linefit_ground_segmentation_ros  # ROS 封装
│   └── pointcloud_to_laserscan         # 点云转激光
└── rm_simulation/                      # 仿真层
    ├── pb_rm_simulation                # Gazebo 世界 + launch
    └── livox_laser_simulation_RO2      # Livox 仿真插件
```

### 5.2 各包功能

| 包名 | 功能 | 说明 |
|------|------|------|
| **livox_ros_driver2** | Livox Mid-360 雷达驱动 | 发布 `CustomMsg` 和 `PointCloud2` |
| **fast_lio** | FAST-LIO 建图算法 | 输出 odom→base_link TF 和点云地图 |
| **point_lio** | Point-LIO 建图算法 | 比 FAST_LIO 频率更高（100+Hz） |
| **icp_registration** | ICP 点云配准定位 | 发布 map→odom TF |
| **rm_navigation** | Nav2 参数配置 + launch | 导航、定位、RViz 启动文件 |
| **fake_vel_transform** | 自旋解耦 + 速度变换 | 发布 base_link→base_link_fake，让 Nav2 不受机器人自旋影响；变换速度指令并注入自旋速度 |
| **teb_local_planner** | TEB 弹性带局部规划器 | 替代默认 DWB 规划器 |
| **teb_msgs** | TEB 消息定义 | 规划器内部通信 |
| **costmap_converter** | 代价地图转换 | 将代价地图转为多边形/点集供 TEB 使用 |
| **costmap_converter_msgs** | costmap_converter 消息定义 | 内部通信 |
| **rm_nav_bringup** | 顶层 launch | 组装所有模块启动 |
| **imu_complementary_filter** | IMU 互补滤波 | 加速度计+陀螺仪融合输出姿态 |
| **linefit_ground_segmentation** | 地面分割算法 | 纯算法库，无 ROS 依赖 |
| **linefit_ground_segmentation_ros** | 地面分割 ROS 封装 | 订阅点云，发布地面过滤后点云 |
| **pointcloud_to_laserscan** | 点云转激光扫描 | 3D→2D，Nav2 需要 2D scan |
| **pb_rm_simulation** | Gazebo 世界模型 | 机器人 URDF、仿真 launch、地图 |
| **ros2_livox_simulation** | Livox 仿真插件 | Gazebo Classic RayPlugin 实现 |

---

## 6. 数据流与依赖关系

### 6.1 数据流

```
Gazebo 仿真环境
  │
  ├─ /livox/lidar (CustomMsg)
  │     ├─→ [pointcloud_to_laserscan] ──→ /scan ──→ Nav2
  │     ├─→ [fast_lio / point_lio] ──→ /odom, TF: map→odom
  │     └─→ [icp_registration] ──→ TF: map→odom (备选)
  │
  ├─ /livox/imu (Imu)
  │     └─→ [imu_complementary_filter]
  │
  ├─ [linefit_ground_segmentation] ──→ 地面过滤后点云
  │
  ├─ [fake_vel_transform] ──→ TF: base_link→base_link_fake
  │
  └─ Nav2 栈
        ├─ [TEB Planner] ← [costmap_converter]
        ├─ [Costmap]
        └─ [BT Navigator]
              │
              ▼
        /cmd_vel_chassis → Gazebo 机器人运动
```

### 6.2 TF 树

```
map
 └─ odom (LIO / ICP / slam_toolbox 发布)
     └─ base_link (LIO 发布)
         ├─ livox_frame (URDF 固定关节)
         ├─ imu_link (URDF 固定关节)
         ├─ wheel_1~4 (URDF 连续关节)
         └─ base_link_fake (fake_vel_transform 发布)
```

### 6.3 关键话题

| Topic | Type | 来源 | 说明 |
|-------|------|------|------|
| `/livox/lidar` | CustomMsg | Gazebo 插件 | Livox 原始点云 |
| `/livox/lidar/pointcloud` | PointCloud2 | 驱动转换 | ROS2 标准点云 |
| `/livox/imu` | Imu | Gazebo 插件 | IMU 数据 |
| `/scan` | LaserScan | pointcloud_to_laserscan | 2D 激光（Nav2 输入） |
| `/odom` | Odometry | LIO | 里程计 |
| `/cmd_vel_chassis` | Twist | Nav2 → Gazebo | 底盘速度控制 |

### 6.4 依赖链

```
livox_ros_driver2          ← 所有 LIO 和 ICP 的输入源
    ├── fast_lio           ← 发布 map→odom TF
    ├── point_lio          ← 发布 map→odom TF（备选）
    └── icp_registration   ← 发布 map→odom TF（备选）

pointcloud_to_laserscan    ← CustomMsg → /scan（Nav2 需要）

rm_navigation              ← Nav2 参数配置
    └── teb_local_planner  ← 局部规划器
        └── costmap_converter ← 代价地图转换

fake_vel_transform         ← 补偿雷达偏心 + 旋转解耦

rm_nav_bringup             ← 把上面所有东西组装到一个 launch 中
```

### 6.5 fake_vel_transform 深度解析

#### 问题背景

RM 赛场上的哨兵机器人需要**持续自旋**（yaw 旋转）来降低敌方命中率。这带来一个导航难题：

- LIO 输出的 `odom → base_link` TF 中，`base_link` 的朝向在**持续变化**
- Nav2 的代价地图（costmap）和规划器都依赖 `base_link` 的朝向来计算障碍物相对位置
- 如果直接用真实的 `base_link`，规划器看到的机器人在不停旋转，**无法稳定规划路径**

#### 解决方案：引入 base_link_fake

创建一个虚拟坐标系 `base_link_fake`，它代表机器人**"应该朝向的方向"**（规划器期望的朝向），而不是实际旋转中的朝向。

```
base_link（真实，持续自旋）
  └─ base_link_fake（虚拟，朝向稳定，供 Nav2 使用）
```

#### 工作机制

`fake_vel_transform` 节点做三件事：

**1. 订阅规划器输出，计算目标朝向差**

```
/local_plan (TEB 输出的局部路径)
  → 取路径 1/4 处的位姿朝向 → teb_angle
  → 当前机器人实际朝向 → base_link_angle（从 odom→base_link TF 获取）
  → current_angle_ = teb_angle - base_link_angle
```

**2. 发布 TF：base_link → base_link_fake（20Hz）**

```
base_link_fake 相对于 base_link 旋转 current_angle_
→ Nav2 看到的 base_link_fake 朝向是稳定的（= 规划器期望的朝向）
→ 代价地图不会因机器人自旋而混乱
```

**3. 速度指令坐标变换**

```
Nav2 输出 /cmd_vel（在 base_link_fake 坐标系下）
  → 通过旋转变换到 base_link 坐标系
  → 强制设置 angular.z = spin_speed（持续自旋速度）
  → 发布 /cmd_vel_chassis（在 base_link 坐标系下，Gazebo 接收）
```

#### 为什么需要这个节点

| 没有 base_link_fake | 有 base_link_fake |
|---------------------|-------------------|
| 代价地图随机器人旋转，障碍物位置不断变化 | 代价地图稳定，障碍物位置固定 |
| 规划器无法生成有效路径 | 规划器正常工作 |
| 机器人朝向和规划方向混淆 | 机器人朝向（自旋）与规划方向解耦 |

#### Nav2 配置中的体现

```yaml
# nav2_params_sim.yaml
robot_base_frame: base_link_fake   # ← 控制器、代价地图都用 fake 帧
# 而 base_link 只用于 URDF TF 和传感器数据
```

所有需要"机器人朝向"的 Nav2 插件（controller_server、costmap、behavior_server）都使用 `base_link_fake`，而 LIO 和传感器数据仍然用真实的 `base_link`。这样就把**自旋运动**和**导航运动**完全解耦了。

### 6.6 costmap_converter 动机分析

#### 问题：Nav2 默认代价地图无法区分动态障碍物

Nav2 默认的 `obstacle_layer` 工作方式是：传感器检测到障碍物 → 将对应栅格标记为占用 → 代价地图更新。关键在于，**它只记录"哪里有障碍物"，而不关心"障碍物是否在移动"**。

当一个动态障碍物（如对方机器人）在地图上移动时：

```
时刻 T1:  障碍物在位置 A → A 被标记为占用
时刻 T2:  障碍物移动到 B → A 和 B 都被标记为占用
时刻 T3:  障碍物移动到 C → A、B、C 都被标记为占用
```

经过一段时间后，障碍物经过的所有路径都会被标记为占用，形成一条**"障碍物拖尾"**。规划器看到的是一片越来越大的禁区，无法规划出绕过动态障碍物的路径。

#### 解决方案：costmap_converter 动态障碍物插件

本项目实现了 `CostmapToDynamicObstacles` 插件，用三层算法分离动态障碍物：

```
Costmap2D 原始栅格
  │
  ▼
BackgroundSubtractor（背景减除）
  │  用双速自适应滤波器区分"背景"（静态）和"前景"（动态）
  │  慢滤波器（α=0.3）: 缓慢学习静态环境
  │  快滤波器（α=0.85）: 快速响应新出现的物体
  │  前景 = 快滤波器 > 阈值 且 (快-慢) > 分离阈值
  │
  ▼
BlobDetector（斑点检测）
  │  在前景掩码中检测连通区域
  │  提取轮廓、质心、面积等特征
  │  按面积/圆形度过滤噪声
  │
  ▼
CTracker（多目标跟踪）
  │  匈牙利算法做数据关联（最小化距离代价矩阵）
  │  卡尔曼滤波做状态估计（位置 + 速度）
  │  维护轨迹 ID，处理遮挡/丢失
  │
  ▼
ObstacleArrayMsg → TEB 规划器
  每个动态障碍物: 位置[m]、速度[m/s]、轮廓多边形、跟踪ID
```

TEB 规划器拿到这些信息后，可以：
- **预测动态障碍物的未来轨迹**（基于卡尔曼滤波估计的速度）
- **在时间维度上优化路径**（TEB 的"T"就是时间维度）
- **生成绕行动作**而不是急停

#### STVL vs costmap_converter 的区别

| 组件 | 作用 | 解决的问题 |
|------|------|------------|
| **costmap_converter** | 检测并跟踪**动态障碍物**，输出位置+速度给 TEB | 动态障碍物拖尾、动态避障 |
| **stvl_layer** (SpatioTemporalVoxelLayer) | 全局代价地图中对过期占用栅格做**时间衰减** | 静态障碍物信息过期（传感器看不到就不更新） |

两者解决不同的问题：STVL 处理的是"看不到的区域太久没更新"，costmap_converter 处理的是"看到的东西在动"。

#### Nav2 配置中的体现

```yaml
# nav2_params_sim.yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "costmap_converter::CostmapToDynamicObstacles"
        enabled: True
        include_dynamic_obstacles: True    # ← 启用动态障碍物检测
        max_transform_tolerance: 0.3

# teb_local_planner 相关参数
  weight_dynamic_obstacle: 10.0            # ← 动态障碍物代价权重
  include_dynamic_obstacles: True
```

#### 为什么需要双速滤波器（而不是单个滤波器）

##### 情况一：新区域扫描带来的"脉冲检测"问题

机器人移动时，代价地图原点随之变化。`transformToCurrentFrame` 会把上一帧的背景模型平移到当前帧坐标系下，但平移后**新暴露的区域没有历史数据**（填充为 0）。

如果新扫描到的区域恰好有一个静态障碍物（值=254）：

**单快滤波器：**
```
帧1: 背景模型=0, 当前帧=254 → 滤波值=216, |差|=216 ✓ 误检为动态
帧2: 滤波值=251, |差|=35    → 还能检测到
帧3: 滤波值=254, |差|=3     → 消失了
```
检测信号是"脉冲式"的——瞬间出现、瞬间消失。这种不稳定的检测结果会被传给 TEB 规划器，导致路径规划抖动。

**双速滤波器：**
```
帧1: 快=216, 慢=76,  差=140  → 检测到
帧2: 快=251, 慢=129, 差=122  → 仍然检测到
帧3: 快=254, 慢=167, 差=87   → 仍然检测到
...
帧10: 快=254, 慢=252, 差=2   → 最终收敛为静态
```
慢滤波器滞后学习，差值缓慢下降而非骤然消失，检测信号平滑稳定。**更重要的是，当差值最终消失时，说明两个滤波器都已确认"这里长期存在障碍物"，可以安全地将其归类为静态障碍物。**

同理，机器人在已探索区域来回移动时，某些静态障碍物会反复进出视野。单快滤波器每次重新看到时都会产生脉冲误检，而双速滤波器的慢滤波器保留了历史记忆，不会因为"重新看到"就判为动态。

##### 情况二：阈值调参的困境

**单滤波器**只有一个调参旋钮——`threshold`：

```
|p(t) - p(t-1)| >= threshold → 动态
```

- threshold 设小（灵敏）→ 噪声和衰减残留容易误检（重影）
- threshold 设大（保守）→ 缓慢移动的障碍物漏检
- alpha 不能太大（需要平滑噪声），也不能太小（响应太慢）
- **一个参数要同时平衡噪声抑制和灵敏度，几乎不可能调好**

**双滤波器**有多个独立可调参数：

| 参数 | 控制什么 | 调参逻辑 |
|------|---------|---------|
| `alpha_fast` | 快滤波器响应速度 | 决定"多快算新出现" |
| `alpha_slow` | 慢滤波器学习速度 | 决定"多久算长期存在" |
| `min_sep` | 快慢差值阈值 | 决定"多大差异算动态" |
| `min_occupancy` | 快滤波器最低响应 | 决定"多强的信号值得检测" |
| `max_occupancy_neighbors` | 邻域密度阈值 | 决定"多密集的区域排除检测" |

这些参数可以独立调整：
- 想检测更慢的移动 → 减小 `alpha_slow`（慢滤波器更慢）
- 想减少噪声误检 → 增大 `min_sep` 或 `min_occupancy`
- 想处理更密集的环境 → 调整 `max_occupancy_neighbors`

**本质上，双滤波器把"是否动态"这个判断从单一阈值变成了一个多维度的信号空间，每个维度可以独立调节，适应不同的环境和需求。**

经过以上三重判定和形态学闭运算，BackgroundSubtractor 输出 `fg_mask_`：一张二值图，白色像素 = 动态前景，黑色像素 = 背景。但这张图只回答了"哪些像素是动态的"，还没有回答"这些像素组成了几个独立的障碍物"——这正是 BlobDetector 要解决的问题。

#### 步骤二：BlobDetector — 从像素到障碍物

BlobDetector 继承自 OpenCV 的 `SimpleBlobDetector`，唯一改动是额外提取轮廓。输入 `fg_mask_`，输出每个障碍物的质心（`KeyPoint`）和轮廓（`contour`）。

**核心流程：**

```
fg_mask_ (二值图)
  │
  ├─ cv::findContours() → 提取所有白色区域的边界轮廓
  │
  ├─ 对每个轮廓逐级过滤:
  │    ① 面积: 3 ≤ area ≤ 300 (排除噪声点和异常大区域)
  │    ② 圆形度: 4π×area/perimeter² ≥ 0.2 (排除线状伪影)
  │    ③ 惯性比: 主轴/次轴惯性矩比 ≥ 0.2 (排除极细长形状)
  │    ④ 凸性: 轮廓面积/凸包面积 (排除严重凹陷)
  │    ⑤ 颜色: 质心像素值 == 255 (确认是前景)
  │
  ├─ 计算通过过滤的轮廓:
  │    质心 = 一阶矩 / 零阶矩
  │    半径 = 轮廓点到质心距离的中位数
  │
  └─ 输出: keypoints (质心+半径) + contours (轮廓点集)
```

BlobDetector 回答的问题是：**前景像素组成了几个独立物体？每个在哪里、多大、什么形状？** 输出传给 CTracker 进行帧间关联。

#### 步骤三：CTracker — 帧间跟踪与速度估计

BlobDetector 每帧输出一组检测结果，但不知道"这一帧的检测 A"和"上一帧的检测 B"是不是同一个物体。CTracker 用**匈牙利算法**做数据关联、**卡尔曼滤波器**做状态估计，维护每个动态障碍物的身份 ID、轨迹历史和速度估计。

##### 3.1 构建代价矩阵

```cpp
distMatrix_t Cost(N * M);  // N=轨迹数, M=检测数
for (每条轨迹 i)
  for (每个检测 j)
    Cost[i + j*N] = tracks[i]->CalcDist(detectedCentroid[j]);
```

`CalcDist` 计算的是**轨迹的卡尔曼预测位置**和**检测质心**之间的欧氏距离。生成 N×M 的距离矩阵，比如 3 条轨迹、2 个检测：

```
         检测0@pos(6,3)   检测1@pos(19,8)
轨迹0  [ 1.2,              15.3  ]    ← 轨迹0 预测位置离检测0 近
轨迹1  [ 7.3,               0.8  ]    ← 轨迹1 预测位置离检测1 近
轨迹2  [ 12.1,             15.3  ]    ← 轨迹2 离两个检测都远
```

##### 3.2 匈牙利算法求解最优配对

```cpp
AssignmentProblemSolver APS;
APS.Solve(Cost, N, M, assignment, AssignmentProblemSolver::optimal);
```

匈牙利算法（Munkres 算法）在代价矩阵中找到**总代价最小的一对一配对**，约束：每条轨迹最多匹配一个检测、每个检测最多匹配一条轨迹。

核心步骤：
```
步骤1: 每行减去该行最小值（行归约）
步骤2: 每列减去该列最小值（列归约）
步骤3: 用最少的横线覆盖所有零元素
步骤4: 线数 = min(N,M) → 找到最优解
        否则 → 找未覆盖元素中的最小值 h，
               覆盖行 +h，未覆盖列 -h，回到步骤3
```

上例的结果：轨迹0↔检测0（代价1.2），轨迹1↔检测1（代价0.8），轨迹2↔无匹配。

##### 3.3 距离阈值过滤

```cpp
if (Cost[i + assignment[i]*N] > dist_thresh) {
    assignment[i] = -1;        // 取消配对
    tracks[i]->skipped_frames = 1;
}
```

即使匈牙利找到了最优配对，距离超过 `dist_thresh` 也认为不是同一物体。这是防止远处两个不相关的障碍物被错误关联的安全阀。

##### 3.4 轨迹生命周期管理

```
未配对的轨迹 → skipped_frames++
skipped_frames > max_allowed_skipped_frames → 删除轨迹（物体已离开视野）

未配对的检测 → 创建新轨迹，分配新 ID（新物体出现）
```

##### 3.5 卡尔曼滤波器 — 状态估计

每条轨迹内部维护一个卡尔曼滤波器，用于估计物体的位置和速度。

**状态空间模型（匀速模型）：**

```
状态向量: [x, y, z, ẋ, ẏ, ż]   → 6 维
测量向量: [x, y, z]               → 3 维

状态转移矩阵 F:
┌          ┐   ┌                ┐   ┌          ┐
│ x(t)     │   │ 1 0 0 dt 0  0 │   │ x(t-1)   │
│ y(t)     │   │ 0 1 0 0  dt 0 │   │ y(t-1)   │
│ z(t)     │ = │ 0 0 1 0  0  dt│ × │ z(t-1)   │
│ ẋ(t)     │   │ 0 0 0 1  0  0 │   │ ẋ(t-1)   │
│ ẏ(t)     │   │ 0 0 0 0  1  0 │   │ ẏ(t-1)   │
│ ż(t)     │   │ 0 0 0 0  0  1 │   │ ż(t-1)   │
└          ┘   └                ┘   └          ┘

物理含义: x(t) = x(t-1) + ẋ(t-1)×dt（匀速直线运动假设）
```

**每帧两步操作：**

```
预测: x̂(t|t-1) = F × x̂(t-1|t-1)     → 用上一帧最优估计预测当前位置
更新: x̂(t|t) = x̂(t|t-1) + K × (z - H × x̂(t|t-1))
      → 有匹配检测时: z = 检测质心，修正位置+速度估计
      → 无匹配检测时: z = 预测位置，按惯性外推（等价于只预测不修正）
```

**噪声参数：**

| 参数 | 值 | 含义 |
|------|-----|------|
| `sigma` (加速度标准差) | 0.5 | 假设物体加速度噪声为 0.5 m/s²。越大→越相信测量值（灵敏但 noisy）；越小→越相信预测（平滑但迟钝） |
| `measurementNoiseCov` | 5e-2 | 测量噪声协方差。反映检测质心的精度 |
| `errorCovPost` | 1e6 | 初始协方差。设为极大值，表示初始时刻高度不确定 |
| `dt` | 0.2 | 时间步长（5Hz）。代价地图更新频率 |

**速度估计输出：**

```cpp
// CTrack::getEstimatedVelocity()
return KF.LastVelocity;  // [px/s]，来自卡尔曼滤波器状态向量的 [ẋ, ẏ, ż]
```

这个速度是卡尔曼滤波器在多次测量修正后收敛的估计值，比直接差分（位置差/时间差）平滑得多，不容易受单帧噪声影响。

##### 3.6 完整跟踪流程图

```
BlobDetector 输出: [{质心A, 轮廓A}, {质心B, 轮廓B}, ...]
      │
      ▼
 CTracker::Update()
      │
      ├─ ① 构建代价矩阵: Cost[i,j] = 轨迹i预测位置 → 检测j质心 的欧氏距离
      │
      ├─ ② 匈牙利算法: 求解最小总代价的一对一配对
      │     轨迹0 ↔ 检测1 (代价 1.2)
      │     轨迹1 ↔ 检测0 (代价 0.8)
      │     轨迹2 ↔ 无    (代价太大)
      │
      ├─ ③ 距离阈值过滤: 代价 > dist_thresh → 取消配对
      │
      ├─ ④ 轨迹生命周期:
      │     未配对轨迹 → skipped_frames++
      │     超过 max_allowed_skipped_frames → 删除（物体离开视野）
      │     未配对检测 → 创建新轨迹（新物体出现）
      │
      └─ ⑤ 卡尔曼滤波器更新:
            有配对 → KF.correct(检测值) → 修正位置 + 更新速度估计
            无配对 → KF.correct(预测值) → 按惯性外推

 输出: tracks 列表，每条轨迹包含:
   track_id         — 唯一标识（用于 TEB 区分不同障碍物）
   prediction       — 预测位置 [px]
   getEstimatedVelocity() — 速度估计 [px/s]
   getLastContour() — 最近轮廓 [px]
   trace            — 历史轨迹点序列
   skipped_frames   — 连续丢失帧数
```

##### 3.7 输出如何被 TEB 使用

```cpp
// costmap_to_dynamic_obstacles.cpp 中的输出组装
for (每条轨迹) {
    // 轮廓: 像素坐标 → 世界坐标[m]
    getContour(i, contour);  // px → m 转换
    // 速度: 像素/秒 → 米/秒
    velocity.x = track->getEstimatedVelocity().x * costmap_resolution;
    velocity.y = track->getEstimatedVelocity().y * costmap_resolution;
    // 组装 ObstacleArrayMsg
    obstacle.velocities.push_back(velocity);
    obstacle.polygon = polygon;
    obstacles->obstacles.push_back(obstacle);
}
```

TEB 规划器拿到每个动态障碍物的**位置、速度、轮廓**后：
- 用速度向量预测障碍物的未来轨迹
- 在时间维度上优化机器人路径（TEB 的 "T" = Time）
- 生成绕行动作而非急停

---

## 7. 仿真启动流程

```
bringup_sim.launch.py
  ├─ rm_simulation.launch.py
  │     ├─ gzserver (Gazebo 物理仿真)
  │     ├─ gzclient (Gazebo GUI)
  │     ├─ spawn_entity.py (加载机器人模型)
  │     ├─ robot_state_publisher (发布 URDF TF)
  │     └─ joint_state_publisher
  │
  ├─ imu_complementary_filter_node
  ├─ linefit_ground_segmentation_node
  ├─ pointcloud_to_laserscan_node
  │
  ├─ LIO Group
  │     ├─ static_transform_publisher (odom→lidar_odom)
  │     └─ fast_lio 或 point_lio
  │
  ├─ Localization Group (仅 mode:=nav)
  │     └─ slam_toolbox 或 amcl 或 icp_registration
  │
  ├─ fake_vel_transform_node
  │
  ├─ slam_toolbox (仅 mode:=mapping)
  │
  └─ bringup_rm_navigation.py
        ├─ component_container_mt
        ├─ navigation_launch.py
        │     ├─ controller_server (TEB)
        │     ├─ planner_server
        │     ├─ behavior_server
        │     ├─ bt_navigator
        │     ├─ waypoint_follower
        │     └─ velocity_smoother
        └─ rviz_launch.py
```

---

## 8. Nav2 参数要点

| 参数 | 值 | 说明 |
|------|-----|------|
| 控制器 | TEB (`teb_local_planner::TebLocalPlannerROS`) | 替代默认 DWB |
| 局部代价地图 | obstacle_layer + inflation_layer | |
| 全局代价地图 | static_layer + stvl_layer + inflation_layer | STVL 时空体素层 |
| base_frame | `base_link_fake` | 由 fake_vel_transform 发布 |
| odom_frame | `odom` | 由 LIO 发布 |
| 恢复行为 | Spin / BackUp / Wait | nav2_behaviors 插件 |

---

## 9. 实车适配要点

| 配置项 | 文件 | 说明 |
|--------|------|------|
| 雷达 IP | `config/reality/MID360_config.json` → `lidar_configs.ip` | Mid-360 网络配置 |
| 雷达-底盘偏移 | `config/reality/measurement_params_real.yaml` | x, y 距离 |
| 雷达离地高度 | `config/reality/segmentation_real.yaml` → `sensor_height` | 影响地面分割 |
| Nav2 参数 | `config/reality/nav2_params_real.yaml` | robot_radius、速度等 |
| 点云保存 | `fastlio_mid360_real.yaml` → `pcd_save_en: true` | 运行时调用 `/map_save` |

---

## 10. 编译顺序（colcon 自动处理）

```
1. livox_ros_driver2 (消息生成)
2. costmap_converter_msgs, teb_msgs (消息生成)
3. costmap_converter (依赖 teb_msgs)
4. teb_local_planner (依赖 costmap_converter)
5. fast_lio, point_lio, icp_registration (依赖 livox_ros_driver2)
6. rm_navigation, fake_vel_transform, rm_nav_bringup
7. rm_perception/* (独立)
8. ros2_livox_simulation, pb_rm_simulation (Gazebo 相关)
```

---

## 11. 常见问题

| 问题 | 原因 | 解决 |
|------|------|------|
| TF 超时 `map frame does not exist` | LIO 未启动或 mode 未指定 | 启动命令加 `mode:=mapping` 或 `mode:=nav` |
| `costmap_converter` clone 失败 | LihanChen2004 fork 已删除 | 改用 `rst-tu-dortmund/costmap_converter` |
| `glog/logging.h` 缺失 | point_lio 依赖 | `apt install libgoogle-glog-dev` |
| `SuiteSparse` 缺失 | teb_local_planner 依赖 | `apt install libsuitesparse-dev` |
| `libg2o` 缺失 | teb_local_planner 依赖 | `apt install libg2o-dev` |
| `dwb_critics` 缺失 | teb_local_planner 依赖 | `apt install ros-humble-nav2-dwb-controller` |
| Docker 内 git 权限问题 | 容器 root 与宿主机用户不一致 | `git config --global --add safe.directory '*'` |
| Docker 内 GUI 不显示 | X11 转发未配置 | `xhost +local:docker` + `-e DISPLAY` |
