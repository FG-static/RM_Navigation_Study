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

#### 解决方案

我们通过引入 base_link_fake来解决上述难题。创建一个虚拟坐标系 `base_link_fake`，它代表机器人应该朝向的方向（规划器期望的朝向），而不是实际旋转中的朝向

```
base_link（真实，持续自旋）
  └─ base_link_fake（虚拟，朝向稳定，供 Nav2 使用）
```

| 没有 base_link_fake | 有 base_link_fake |
|---------------------|-------------------|
| 代价地图随机器人旋转，障碍物位置不断变化 | 代价地图稳定，障碍物位置固定 |
| 规划器无法生成有效路径 | 规划器正常工作 |
| 机器人朝向和规划方向混淆 | 机器人朝向（自旋）与规划方向解耦 |

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

所有需要"机器人朝向"的 Nav2 插件（controller_server、costmap、behavior_server）都使用 `base_link_fake`，而 LIO 和传感器数据仍然用真实的 `base_link`。这样就把**自旋运动**和**导航运动**完全解耦了

### 6.6 costmap_converter 动机分析

#### 问题背景

Nav2 默认的 `obstacle_layer` 工作方式是：传感器检测到障碍物 → 将对应栅格标记为占用 → 代价地图更新。关键在于，**它只记录"哪里有障碍物"，而不关心"障碍物是否在移动"**

当一个动态障碍物（如对方机器人）在地图上移动时：

```
T1:  障碍物在位置 A → A 被标记为占用
T2:  障碍物移动到 B → A 和 B 都被标记为占用
T3:  障碍物移动到 C → A、B、C 都被标记为占用
```

经过一段时间后，障碍物经过的所有路径都会被标记为占用，形成一条“障碍物拖尾”。规划器看到的是一片越来越大的禁区，无法规划出绕过动态障碍物的路径

#### 解决方案

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

TEB 规划器拿到这些信息后，可以预测动态障碍物的未来轨迹（基于卡尔曼滤波估计的速度）

#### STVL vs costmap_converter 的区别

| 组件 | 作用 | 解决的问题 |
|------|------|------------|
| **costmap_converter** | 检测并跟踪**动态障碍物**，输出位置+速度给 TEB | 动态障碍物拖尾、动态避障 |
| **stvl_layer** (SpatioTemporalVoxelLayer) | 全局代价地图中对过期占用栅格做**时间衰减** | 静态障碍物信息过期（传感器看不到就不更新） |

两者解决不同的问题：STVL 处理的是"看不到的区域太久没更新"，costmap_converter 处理的是"看到的东西在动"

#### 为什么需要双速滤波器（而不是单个滤波器）

##### 情况一：新区域扫描带来的"脉冲检测"问题

机器人移动时，代价地图原点随之变化。`transformToCurrentFrame` 会把上一帧的背景模型平移到当前帧坐标系下，但平移后**新暴露的区域没有历史数据**（填充为 0）

如果新扫描到的区域恰好有一个静态障碍物（值=254）：

**单快滤波器：**
```
帧1: 背景模型=0, 当前帧=254 → 滤波值=216, |差|=216 ✓ 误检为动态
帧2: 滤波值=251, |差|=35    → 还能检测到
帧3: 滤波值=254, |差|=3     → 消失了
```
检测信号是"脉冲式"的——瞬间出现、瞬间消失。这种不稳定的检测结果会被传给 TEB 规划器，导致路径规划抖动

**双速滤波器：**
```
帧1: 快=216, 慢=76,  差=140  → 检测到
帧2: 快=251, 慢=129, 差=122  → 仍然检测到
帧3: 快=254, 慢=167, 差=87   → 仍然检测到
...
帧10: 快=254, 慢=252, 差=2   → 最终收敛为静态
```
慢滤波器滞后学习，差值缓慢下降而非骤然消失，检测信号平滑稳定。**更重要的是，当差值最终消失时，说明两个滤波器都已确认"这里长期存在障碍物"，可以安全地将其归类为静态障碍物**

同理，机器人在已探索区域来回移动时，某些静态障碍物会反复进出视野。单快滤波器每次重新看到时都会产生脉冲误检，而双速滤波器的慢滤波器保留了历史记忆，不会因为"重新看到"就判为动态

##### 情况二：阈值调参

**单滤波器**只有一个调参旋钮——`threshold`：

```
|p(t) - p(t-1)| >= threshold → 动态
```

- threshold 设小（灵敏）→ 噪声和衰减残留容易误检（重影）
- threshold 设大（保守）→ 缓慢移动的障碍物漏检
- alpha 不能太大（需要平滑噪声），也不能太小（响应太慢）

一个参数要同时平衡噪声抑制和灵敏度，几乎不可能调好

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

**本质上，双滤波器把"是否动态"这个判断从单一阈值变成了一个多维度的信号空间，每个维度可以独立调节，适应不同的环境和需求**

经过以上三重判定和形态学闭运算，BackgroundSubtractor 输出 `fg_mask_`：一张二值图，白色像素 = 动态前景，黑色像素 = 背景。但这张图只回答了"哪些像素是动态的"，还没有回答"这些像素组成了几个独立的障碍物"——这正是 BlobDetector 要解决的问题

#### 步骤二：BlobDetector — 从像素到障碍物

BlobDetector 继承自 OpenCV 的 `SimpleBlobDetector`，唯一改动是额外提取轮廓。输入 `fg_mask_`，输出每个障碍物的质心（`KeyPoint`）和轮廓（`contour`）

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

BlobDetector 回答的问题是：**前景像素组成了几个独立物体？每个在哪里、多大、什么形状？** 输出传给 CTracker 进行帧间关联

#### 步骤三：CTracker — 帧间跟踪与速度估计

BlobDetector 每帧输出一组检测结果，但不知道"这一帧的检测 A"和"上一帧的检测 B"是不是同一个物体，CTracker 用**匈牙利算法**做数据关联、**卡尔曼滤波器**做状态估计，维护每个动态障碍物的身份 ID、轨迹历史和速度估计

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

匈牙利算法（Munkres 算法）在代价矩阵中找到**总代价最小的一对一配对**，约束：每条轨迹最多匹配一个检测、每个检测最多匹配一条轨迹

核心步骤：
```
步骤1: 每行减去该行最小值（行归约）
步骤2: 每列减去该列最小值（列归约）
步骤3: 用最少的横线覆盖所有零元素
步骤4: 线数 = min(N,M) → 找到最优解
        否则 → 找未覆盖元素中的最小值 h，
               覆盖行 +h，未覆盖列 -h，回到步骤3
```

上例的结果：轨迹0$\leftrightarrow$检测0（代价1.2），轨迹1$\leftrightarrow$检测1（代价0.8），轨迹2$\leftrightarrow$无匹配

##### 3.3 距离阈值过滤

```cpp
if (Cost[i + assignment[i]*N] > dist_thresh) {
    assignment[i] = -1;        // 取消配对
    tracks[i]->skipped_frames = 1;
}
```

即使匈牙利找到了最优配对，距离超过 `dist_thresh` 也认为不是同一物体。这是防止远处两个不相关的障碍物被错误关联的安全阀。

##### 3.4 卡尔曼滤波器 — 状态估计

每条轨迹内部维护一个卡尔曼滤波器，用于估计物体的位置和速度。

**状态空间模型（匀速模型）：**

状态向量: $[x, y, z, ẋ, ẏ, ż]^T$共 6 维
测量向量: $[x, y, z]^T$共 3 维

状态转移：
$$\begin{bmatrix}
  x(t) \\
  y(t) \\
  z(t) \\
  \dot{x}(t) \\
  \dot{y}(t) \\
  \dot{z}(t)
\end{bmatrix} = \boldsymbol{F} \begin{bmatrix}
  x(t - 1) \\
  y(t - 1) \\
  z(t - 1) \\
  \dot{x}(t - 1) \\
  \dot{y}(t - 1) \\
  \dot{z}(t - 1)
\end{bmatrix} = \begin{bmatrix}
  1 & 0 & 0 & dt & 0 & 0 \\
  0 & 1 & 0 & 0 & dt & 0 \\
  0 & 0 & 1 & 0 & 0 & dt \\
  0 & 0 & 0 & 1 & 0 & 0 \\
  0 & 0 & 0 & 0 & 1 & 0 \\
  0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix} \begin{bmatrix}
  x(t) \\
  y(t) \\
  z(t) \\
  \dot{x}(t) \\
  \dot{y}(t) \\
  \dot{z}(t)
\end{bmatrix}$$

物理含义: $x(t) = x(t-1) + ẋ(t-1)×dt$（匀速直线运动假设）


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

##### 3.5 完整跟踪流程图

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

##### 3.6 输出如何被 TEB 使用

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

### 6.7 imu_complementary_filter — 互补滤波器

IMU 互补滤波器用于融合陀螺仪和加速度计数据，输出稳定的姿态四元数。比卡尔曼滤波简单得多——没有协方差矩阵、没有状态转移模型，核心只是一个四元数插值

#### 为什么需要融合

| 传感器 | 短期 | 长期 |
|--------|------|------|
| 陀螺仪 | 精确（角速度积分反映真实旋转） | 漂移（积分误差累积） |
| 加速度计 | 有噪声（振动、运动加速度干扰） | 稳定（长期平均反映重力方向） |

互补滤波器 = 用陀螺仪做短期预测 + 用加速度计做长期修正，两者互补

#### 三步工作流程

```
步骤一：预测（陀螺仪积分）
  q_pred = q(t-1) + 0.5×dt × q(t-1) ⊗ ω
  → 用角速度 ω 把上一帧姿态往前推一步

步骤二：修正（加速度计校正）
  dq_acc = 旋转"预测重力方向"到"测量重力方向"的四元数
  → 只能修正 roll/pitch，无法修正 yaw（重力绕竖直轴旋转不变）

步骤三：融合（四元数球面插值）
  q(t) = q_pred ⊗ slerp(I, dq_acc, gain)
  gain = 0.01 → 99% 信任陀螺仪，1% 信任加速度计
```

#### 与卡尔曼滤波对比

| | 互补滤波 | 卡尔曼滤波 |
|--|---------|-----------|
| 融合方式 | 固定增益插值（gain=0.01） | 自适应卡尔曼增益 K |
| 计算量 | 极低（几次乘法 + 一次 slerp） | 较高（矩阵求逆） |
| 调参 | 1 个参数（gain） | 多个参数（Q, R, P₀） |

#### 额外功能

- **零偏估计**：机器人静止时（加速度≈9.81、角速度变化小），用指数移动平均缓慢更新陀螺仪零偏
- **自适应增益**：剧烈运动时（加速度偏离 9.81 较大），自动减小加速度计权重，避免运动加速度污染姿态

### 6.8 linefit_ground_segmentation — 地面分割算法

3D 点云包含地面和障碍物，Nav2 只需要障碍物。地面分割算法将点云分为地面点和障碍物点，核心思路是：在极坐标下对每个扇区拟合地面线，点到地面线的距离近就是地面，远就是障碍物。

#### 数据结构

| 类 | 角色 | 职责 |
|---|------|------|
| **Bin** | 工具类 | 每个径向格一个 Bin，追踪该格中 z 值最小的点（`addPoint` 只在 z 更小时更新） |
| **Segment** | 数据+算法类 | 每个扇区一个 Segment，包含一组 Bin，实现线拟合（`fitSegmentLines`）和距离计算（`verticalDistanceToLine`） |
| **GroundSegmentation** | 执行者 | 拥有所有 Segment，协调整个流程，管理多线程 |
| typedefs | 定义 | 数据结构别名，不重要 |
| viewer | 可视化 | PCL 可视化调试工具，不重要 |

#### 完整流程

`GroundSegmentation::segment()` 是入口函数，依次执行三步：

```
点云输入 (PointCloud)
      │
      ▼
步骤一: insertPoints — 极坐标分箱（扇区）
  对每个点 (x, y, z):
    range = sqrt(x² + y²)
    angle = atan2(y, x)
    bin_index   = (range - r_min) / bin_step
    segment_index = (angle + π) / segment_step
    segments_[segment_index][bin_index].addPoint(range, z)
    → 每个 Bin 只保留最低 z 点

      │
      ▼
步骤二: getLines → lineFitThread → Segment::fitSegmentLines — 地面线拟合
  对每个扇区，从近到远遍历其所有 Bin 的最低 z 点:
    收集连续的最低 z 点 → 最小二乘拟合线段 (z = slope × d + intercept)
    如果新点偏离当前线太多（error > max_error）
      或坡度超出范围（slope > max_slope 或 < min_slope）
      → 截断当前线段，保存，开始新线段
  输出: 每个扇区若干条地面线段

      │
      ▼
步骤三: assignCluster — 分类
  对每个点:
    找到其所在扇区的地面线
    计算点到线的垂直距离 dist
    dist < max_dist_to_line → 地面点 (label=1)
    dist ≥ max_dist_to_line → 障碍物点 (label=0)
    （如果当前扇区没找到匹配线，还会搜索相邻扇区）

      │
      ▼
输出: segmentation[i] = 1(地面) 或 0(障碍物)
```

#### 线拟合方法

`Segment::fitLocalLine()` 用 Eigen 做最小二乘：

```
X = [[d1, 1], [d2, 1], ...]     (d = 径向距离)
Y = [z1, z2, ...]                (z = 高度)
解: [slope, intercept] = X \ Y   (QR 分解)
```

拟合过程中逐步从近到远扩展线段，当拟合误差超过 `max_error` 或坡度超出 `[min_slope, max_slope]` 时截断，保存当前线段并开始新的。这保证了地面线能适应坡度变化

#### 极坐标分箱示意

```
        90°
         │
  135° ──┼── 45°    ← 360 个扇区 (n_segments)
         │
  180° ──┼── 0°
         │
  225° ──┼── 315°
         │
        270°

  每个扇区内:
  ┌──────────────────────────────────┐
  │  bin0  bin1  bin2  ...  bin119   │ ← 120 个径向格 (n_bins)
  │  近                          远   │
  │  r_min ────────────────- r_max   │
  │  每个 bin 只存最低 z 点            │
  └──────────────────────────────────┘
```
简单来说，扇区从极坐标系 $\rho$ 轴方向分隔，`bin`（径向格）从极坐标系 $\theta$ 轴方向分隔

### 6.9 pointcloud_to_laserscan — 3D 点云转 2D 激光

Nav2 的代价地图基于 2D LaserScan，不接受 3D 点云。这个节点把障碍物点云投影成 2D 激光扫描

```
输入: 3D PointCloud2 (x, y, z)  ← 来自地面分割的 /segmentation/obstacle
输出: 2D LaserScan (angle, range)

对每个点:
  ① 高度过滤: z 不在 [min_height, max_height] → 丢弃
  ② 距离过滤: sqrt(x²+y²) 不在 [range_min, range_max] → 丢弃
  ③ 角度计算: angle = atan2(y, x)
  ④ 分配到对应角度 bin，同一 bin 保留最近的点
```

**z 维度在过滤后被丢弃**，输出的 LaserScan 只有 (angle, range) 二维信息，z=0。这意味着 3D→2D 是不可逆的——`laserscan_to_pointcloud` 虽然可以把 LaserScan 转回 PointCloud2，z 全部为 0，无法恢复原始高度信息

本项目配置（`pointcloud_to_laserscan_launch.py`）：
- 输入：`/segmentation/obstacle`（地面分割后的障碍物点云）
- 输出：`/scan`（Nav2 costmap 的标准输入）
- 角度范围：-π ~ π（360°）
- 高度过滤：-1.0m ~ 0.1m
- 距离范围：0.45m ~ 10.0m

### 6.10 感知管线总结

三个感知子包组成一条从原始传感器数据到 Nav2 可用输入的串行管线：

```
/livox/lidar/pointcloud (Livox Mid-360 原始 3D 点云)
         │
         ├─────────────────────────────────────────────┐
         │                                             │
         ▼                                             ▼
  [linefit_ground_segmentation]              [imu_complementary_filter]
  地面分割，去除地面点                         IMU 互补滤波，融合陀螺仪+加速度计
         │                                             │
         ├─→ /segmentation/ground (地面点)              ▼
         │                                    /imu/data (姿态四元数)
         └─→ /segmentation/obstacle (障碍物点云)            │
                     │                                     │
                     ▼                                     ▼
            [pointcloud_to_laserscan]              [FAST_LIO / Point_LIO]
            3D→2D 投影，生成 LaserScan              使用 IMU + 点云做里程计
                     │
                     ▼
                   /scan (2D LaserScan)
                     │
                     ▼
                  Nav2 costmap
```

数据流关系：
1. **Livox 驱动**发布原始 3D 点云和 IMU 数据
2. **imu_complementary_filter** 融合 IMU 数据，输出姿态 → 给 LIO 使用
3. **linefit_ground_segmentation** 去除地面，输出纯障碍物点云
4. **pointcloud_to_laserscan** 把 3D 障碍物点云投影为 2D LaserScan → 给 Nav2 使用

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
