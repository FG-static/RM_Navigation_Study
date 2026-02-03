## $$\mathbf{Nav2}~学习笔记$$
### 安装与配置
#### 示例包
使用
```bash
sudo apt install ros-jazzy-navigation2
sudo apt install ros-jazzy-nav2-bringup
```
可安装jazzy版本（可安装其他版本）下的nav2的示例包，使用
```bash
sudo apt install ros-jazzy-turtlebot3*
```
可安装$\mathbf{Turtlebot~3}$，然后用
```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/jazzy/share/turtlebot3_gazebo/models
```
配置好环境变量（第一行如果配置ros时用过就不需要再用了），然后用
```bash
ros2 launch nav2_bringup tb3_simulation_launch.py
```
就能启动运行了
启动后应该能看到这样的画面
![alt text](Image//image.png)
一般来说选择左下角的`Startup`就能启动显示这个画面
然后依次选择上方的`2D Pose Estimate`进行放置指定位置的机器人并扫描整个平面，然后点击`Nav2 Goal`（也有`Navigation2 Goal`的版本）指定接下来机器人移动的目标和最终朝向
![alt text](Image//image-2.png)
途中的一团复杂的彩色线段堆砌的位置就是机器人所在处，巨大的绿色箭头的起点是机器人移动的目标点，方向是机器人最终朝向，松手后机器人就会自动寻路移动

#### 安装与编译
##### 安装
在$\mathbf{Ubuntu20.04}$以上的版本，使用
```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-turtlebot3*
```
来安装nav2
##### 编译
安装好后需要进行编译
先创建一个名为`nav2_ws`的工作空间，进到其下的`src`文件夹内，随后将nav2分支功能包克隆到这个目录下：（在humble或foxy版本中需要将`main`改为对应的`<ros2-distro>-devel`）
```bash
git clone https://github.com/ros-planning/navigation2.git --branch main
```
然后退回到`nav2_ws`目录下，用rosdep获取所有依赖项
```bash
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro jazzy
```
然后使用
```bash
colcon build --symlink-install
```
进行软链接编译
（注意，直接进行编译可能会导致并行编译强制跑满所有内存和cpu直接卡死重启，为了避免这种情况，可以考虑设置交换内存为16~32G，同时进行限制性编译（即限制编译时只能一次编译一个项目等）例如：
```bash
MAKEFLAGS="-j1" colcon build --symlink-install --executor sequential --parallel-workers 1
```
这意味着一次进行一个项目编译，绝不多占用，这样不会导致卡死，但是仍有一个问题就是可能会出现编译器无法识别到`.hpp`文件的情况（明明存在且找到），这时候最好直接安装二进制版，不再折腾此编译，这里我们以二进制版本继续）
输入
```bash
rm -rf ~/nav2_ws/src/navigation2
```
可以删除之前编译的源代码，使用
```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-turtlebot3-gazebo
```
安装二进制版
同时再使用
```bash
sudo apt install ros-jazzy-nav2-loopback-sim ros-jazzy-nav2-bringup ros-jazzy-turtlebot3-gazebo
```
安装```nav2_loopback_sim```，因为jazzy将nav2拆分的更细，光靠上面的无法安装全部
（如果是在折腾全部编译中途传过来二进制版，则很可能遇到环境污染，此时需要删除编译产物
```bash
cd ~/nav2_ws
rm -rf build/ install/ log/
```
记得设置环境变来嗯
```bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/jazzy/share/turtlebot3_gazebo/models
```
）
随后使用```ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False```就能启动仿真了
如果不走二进制安装，且实在无法编译源码，可以使用$\mathbf{Docker}$容器，这里不详细展开

### 导航相关概念
#### 生命周期节点
该节点为ros2独有的节点，和普通节点（`rclcpp::Node`）不一样，生命周期节点（`rclcpp_lifecycle::LifecycleNode`）引入了状态机，它被划分为
- **未配置**$-\mathbf{Unconfigured}$
- **未激活**$-\mathbf{Inactive}$
- **激活**$-\mathbf{Active}$
- **已结束**$-\mathbf{Finalized}$

四个状态
同时必须通过外部指令（或者launch）手动触发它的状态转换
|          |普通节点                |生命周期节点                  |
|----------|----------------------|-----------------------------|
|逻辑执行时间|构造函数完成            |进入Active状态                |
|配置灵活性  |运行时修改参数较麻烦     |可在Inactive状态下完成配置再激活|
|确定性     |低（节点启动顺序不确定等）|高                           |

在nav2中，我们可以使用`nav2_util LifecycleNode`这个包装器，它统一了原有的复杂逻辑并简化了操作，同时它还拥有一个`bond`，这个东西用于连接**生命周期管理器**与各个导航服务，如果`bond`断开，管理器认为服务器不一定处于Active状态，无法信任，会直接**向下过渡**，让所有的导航节点都变为Inactive或者Unconfigured状态

#### 有限状态机($\mathbf{Finite~State~Machine,~FSM}$)
状态机是一种数学模型，描述了一个对象在其生命周期内所经历的状态，以及由于触发事件而导致的状态转换
有限状态机-FSM就是有限个状态，包含
- 有限个状态
- 转换条件
- 状态动作

它的局限性是当逻辑变得复杂，异常情况会增多，难以维护

#### 行为树($\mathbf{Behaviour~Tree,~BT}$)
在这里仅简要介绍，具体可阅读《Behaviour Tree in Robotics and AI》（https://arxiv.org/abs/1709.00084）
假如有若干个状态，FSM就像是网状结构，状态就是各个网络中的节点，自由切换，但是逻辑复杂起来就异常庞大，难以维护
而BT就是一棵树，状态就是各个节点，例如有四个状态A,B,C,D，其中B，D是A的子节点，C是B的子节点
当切换状态时，A会每秒发出多次信号脉冲（$\mathbf{Tick}$），这个tick会一直往下深探，直到到达某个节点后它返回**成功**或者**失败**，如果失败则往回走，如果成功则停下来，若这个节点只是一个**策略**，那么就直接继续走，在这里它经过B到达C，此时B根据C返回的信号决定是否回到A还是继续走或者停止继续
举个例子：
假如A是一个选择节点，B是走路，D是原地自旋，C是前进
A先发射tick到B，经过B到达C，如果确实可以前进，那就在这里停止，否则就返回失败信号，此时A知道B这个路走不通，就去D，再重复以上逻辑
Nav2使用`Behaviour CPP V3`作为行为树库，它可以加载子树，也就是说可以将一个nav2项目的BT加载到另一个更高级别的BT中，从而将其作为插件，同时它还提供了一个NavigationToPoseAction插件，可以从客户端应用程序通过通常的动作接口调用Nav2软件堆栈

#### 导航服务器
规划器和控制器是导航任务的核心，恢复器用于使机器人摆脱不良状态或尝试处理各种形式的问题，以使系统具有容错能力

##### 规划器、控制器和恢复器
在ros2中，这三个服务器以Action Server形式存在，并由BT统一管理调用
它们都托管于一个**地图算法插件**，即**代价地图**$-\mathbf{Costmap~2D}$（被实现为pluginlib插件）及其相关插件层
- 地图算法插件
  所有的导航服务器都托管于代价地图上，规划器托管于**全局代价地图**（关注整张地图），控制器托管于**局部代价地图**（关注机器人周边局部地图）
  这个插件则是代价地图的层级结构，分为
  - **静态层**
    提供自地图的`.yaml`文件，记录的是死物
  - **障碍物层**
    来自激光雷达或者摄像机，捕捉的是活物或动物
  - **膨胀层**
    给障碍物画一个警戒圈，警示机器人与墙的距离

规划器和控制器都需要在运行时配置一个配置名称（任务别名）和使用的算法类型（已注册的插件库的名称）
例如使用名为`FollowPath`的DWB控制器，此时DWB的所有参数都会放置在该命名空间中，例`FollowPath.<param>`
然后这两个服务器会公开任务对应的操作接口，以便调用所选算法
对于恢复器，它类似于备选方案，即用于给机器人不同方案执行后出现问题后的备选方案，其公共接口一般是原地旋转、等待、后退、清理代价图层等
它主要由BT监控并调用，而不是用户

#### 航点跟随
航点跟随是导航系统的基本功能之一，它会告知系统如何使用导航程序到达多个目的地
`nav2_waypoint_follower`软件包含一个航路点和跟踪程序，它一般有两种主要的实现思想，即哑机器人应用程序+智能集中式调度器和智能机器人应用程序+哑集中式调度器
区别在于统一由调度器考虑所有信息还是分给各部分分别考虑

#### 状态估计
在导航项目中，需要提供两个主要的坐标转换：
- map 到 odom 的坐标变换由定位系统（定位，建图，SLAM）提供
- odom 到 base_link 的坐标转换由里程计系统提供

$\mathbf{REP-105}$(https://www.ros.org/reps/rep-0105.html)表示至少必须为机器人建造一个包含``map`` -> `odom` -> `base_link` -> `[sensor frames]/base_laser` 的完整的TF树，TF2是ROS 2中的时变坐标变换库，Nav2使用TF2来表达和获取时间同步的坐标变换
全球定位系统 (GPS、SLAM-同步定位与建图、动作捕捉Motion Capture) 的工作是至少要提供 `map` -> `odom` 的坐标转换
然后，里程计系统的作用是提供 `odom` -> `base_link` 的坐标转化
关于 base_link 的其余坐标转换应该是静态的，并应在 URDF 中定义
`map`是全局地图坐标系，`odom`是里程计坐标系，`base_link`是机器人中心坐标系，`base_laser`是传感器坐标系
这个`map` -> `odom` -> `base_link` -> `base_laser`转换实际上是
`base_laser`传感获取数据，处理为`base_link`坐标系下的数据，再处理为给`odom`坐标系下使用的数据，最后处理为`map`即整张地图的数据（类似于一个坐标）
这样的处理就是为了给导航服务器等其他东西使用的，用于规划和执行机器人运动方案
为什么要多一个odom？为什么不直接把转换到odom这一步的数据直接当成map处理？
因为里程计有误差，从odom到map这一步转换通过AMCL-**自适应蒙特卡洛定位**插件得出，即一个微小的数据修正处理
最后得到的map才是精准的数据

#### 地图过滤器
我们可以对地图文件进行**注释**，即可以将某些区域排除在外避免在这些区域内进行路径规划，这种带注释的地图称为$\mathbf{Filter~mask}$
这个地图过滤器基于成本地图插件实现

### 配置一个机器人
#### urdf
使用urdf可以快速建模一个机器人，语法和`.xml`是一样的，本节内容在`Ros2 for learning`中已学习，假设我们已经有一个urdf建模的机器人，它拥有四个轮子，长方体身体，一个雷达
切记定义一个机器人需要定义它的`visual`、`collision`，如果需要更为专业，需要额外定义`inertial`，关于这个标签-物理属性的惯性 的相关公式，可以直接网上查询到
同时各个关节之间也需要标签`joint`链接，这个标签可以用于发布静态tf转换（注意`odom` -> `base_link`是动态转换）

#### 里程计系统
我们可以从各种传感器硬件中获取里程计信息，如IMU($\mathbf{Inertial~Measurement~Unit}$)-惯性测量单元、LIDAR、RADAR等，odom框架与之相关
里程计系统主要用于解决机器人局部的运动问题（局部的轮胎打滑、运动漂移、运动不稳等）
这个`odom` -> `base_link`的转换一般由tf2或诸如`robot_localization`等的框架发布
这个`robot_localization`会订阅各个有关机器人位姿和运动参数（传感器IMU、车轮编码器等），通过一系列矫正算法（EKF、UKF等）计算出最合适的转换并发布，和`map` -> `odom`不同的是，这个转换着重于在整个地图上的精确位置，而这个`robot_localization`则是关注于机器人的局部
![alt text](Image//image-3.png)
在编写仿真机器人时，其核心代码（一般由python或c++编写的ros2节点代码）应具备TF2的`odom` -> `base_link`的转换以及也要发送消息至Nav2的`nav_msgs/Odometry`，它应包含以下信息：
```bash
# This represents estimates of position and velocity in free space.
# The pose in this message should be specified in the coordinate frame given by header.frame_id
# The twist in this message should be specified in the coordinate frame given by the child_frame_id

# Includes the frame id of the pose parent.
std_msgs/Header header

# Frame id the pose is pointing at. The twist is in this coordinate frame.
string child_frame_id

# Estimated pose that is typically relative to a fixed world frame.
geometry_msgs/PoseWithCovariance pose

# Estimated linear and angular velocity relative to child_frame_id.
geometry_msgs/TwistWithCovariance twist
```
这个信息是发给规划器的，而前面发布的tf转换是给rviz看的
我们还可以设置这个类型为`nav_msgs::msg::Odometry`发布到规划器的信息`odom`中的协方差矩阵参数`odom.pose.covariance[i]`，在其对角线上的部分位置设置为$1\mathrm{e}-9$表示更相信模拟数据
最后注意整个tf树应该是`odom` -> `base_footprint` -> `base_link` -> `other`
注意不要在`.urdf`里写错`<parent link>`和`<child link>`

#### $\mathbf{Gazebo}$
在进入下一步前，可以先在urdf中集成gazebo插件，它可以接管我们原本写的核心框架中负责**监听`/cmd_vel`并发布`odom`和TF**的工作
既然是在urdf中集成，那我们需要直接修改`.urdf.xacro`（如果没有使用xacro则修改`.urdf`），在`</robot>`标签前添加：
```xml
<gazebo>
    <plugin
        filename="gz-sim-diff-drive-system"
        name="gz::sim::systems::DiffDrive">
        <left_joint>front_left_wheel_joint</left_joint> 
        <right_joint>front_right_wheel_joint</right_joint>

        <wheel_separation>${chassis_width + wheel_width}</wheel_separation>
        <wheel_radius>${wheel_radius}</wheel_radius>

        <topic>cmd_vel</topic>
        <odom_topic>odom</odom_topic>
        <frame_id>odom</frame_id>
        <child_frame_id>base_footprint</child_frame_id>
        <publish_odom_tf>true</publish_odom_tf>
    </plugin>

    <plugin
        filename="gz-sim-joint-state-publisher-system"
        name="gz::sim::systems::JointStatePublisher">
        <topic>joint_states</topic>
    </plugin>
</gazebo>
```
其中在humble版本下会有`<gazebo reference="link_name">`即包含的内容表示重新识别机器人在仿真环境中的颜色（因为gazebo使用不同的渲染引擎不识别urdf中的`<material>`标签），这个标签内还可定义摩擦力（如`<mu1>`等标签），在jazzy版gazebo进行了大更新，许多旧版的规则不适用，目前不用写这个标签的原因可能是渲染引擎已经可以自动识别
随后的`<gazebo>`标签内的内容主要是把这个机器人的属性添加到gazebo中，让gazebo“认识”这个机器人，最后的
```xml
<topic>cmd_vel</topic>
<odom_topic>odom</odom_topic>
<tf_topic>/tf</tf_topic>
<frame_id>odom</frame_id>
<child_frame_id>base_footprint</child_frame_id>
<publish_odom_tf>true</publish_odom_tf>
<use_gz_time>true</use_gz_time>
```
分别表明订阅速度的话题名称？里程计的话题名称？tf树话题名称？里程计的坐标系名称？里程计要转换到的机器人基准点坐标系名称？以及是否发布tf转换？是否使用gazebo仿真时间？
配置好后gazebo会代替原本核心框架中的三件事：
- 订阅`/cmd_vel`并发布速度，它会根据物理学推导最终机器人的位姿
- 物理仿真
- 发布里程计话题`/odom`和TF变换（`odom` -> `base_footprint`）

之后还需要写一个新的launch文件，负责启动robot state publisher、Gazebo、调用`spawn_entity`节点-负责将机器人模型添加到gazebo里、bridge桥接节点-桥接gazebo和ros2的话题（gazebo内部使用$\mathbf{gz~transport}$协议，而ros2内部使用$\mathbf{DDS}$协议，这样会导致ros2无法通过`ros2 topic list`等获取数据）
（注意：使用jazzy下的gazebo应提前用
```bash
sudo apt install ros-jazzy-ros-gz
```
安装gazebo依赖，同时`spawn_entity`节点在新版gazebo下的可执行程序变为了`create`，位于`/opt/ros/jazzy/lib/ros_gz_sim`，而gazebo的launch文件变为了`gz_sim.launch.py`，位于`/opt/ros/jazzy/share/ros_gz_sim/launch`，以及最重要的一点，必须先提前为gazebo设定好预设场景，否则一开始机器人就往下掉，因为没有地板，可以使用`empty.sdf`或`shapes.sdf`）

#### 配置传感器
在开始配置传感器前，先在urdf中建模出雷达，同时也要给他各种物理属性（碰撞箱和惯性）
随后我们在`laser_joint`的`</joint>`标签后添加
```xml
<gazebo reference="laser_link">
    <sensor name="gpu_lidar" type="gpu_lidar">
        <topic>scan</topic>
        <pose>0 0 0 0 0 0</pose>
        <frame_id>laser_link</frame_id>
        <gz_frame_id>laser_link</gz_frame_id>
        <visualize>true</visualize> <update_rate>10</update_rate>
        <lidar>
            <scan>
                <horizontal>
                    <samples>360</samples>
                    <resolution>1</resolution>
                    <min_angle>-3.14159</min_angle>
                    <max_angle>3.14159</max_angle>
                </horizontal>
            </scan>
            <range>
                <min>0.12</min>
                <max>12.0</max>
                <resolution>0.01</resolution>
            </range>
        </lidar>
        <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
            <render_engine>ogre2</render_engine>
        </plugin>
    </sensor>
</gazebo>
```
这段代码表明雷达是一个`gpu_lidar`类型（利用显卡扫描而不是cpu）的雷达，第一个`<topic>`标签用于告诉gazebo将扫描的信息发到`/scan`话题，否则它会发到自己的一个长的话题，后面的`<pose>`标签则分别表示相对于原`laser_link`分别在`x y z r p y`上的偏移量，这里的`<frame_id>`和`<gz_frame_id>`标签可能很重要，它告诉rviz雷达消息来源于这个坐标系，`<visualize>`标签表示显示扫描光束，下一个标签明显是扫描频率（单位是$\mathrm{Hz}$）
之后`<scan>`标签里的内容是2D扫描参数
- `<samples>`
  表示一圈扫描产生360个采样点
- `<min/max_angle>`
  表示扫描范围，这里是全方位扫描

之后的`<range>`标签表示测距范围，里面的第三个标签表示传感器能识别$1\mathrm{cm}$级别的距离变化
后面就是插件加载标签了，`orge2`是指定的渲染引擎
这里时间戳同步非常重要，请注意启动rviz2时它的时间戳是仿真时间内的（启动左下角看时间），同时也要注意tf2 monitor给出的时间戳延迟（Net Delay）最好为0
我们也可以添加一个摄像头，用来捕捉图像，只需要在urdf的`<gazebo>`标签下建立和`<sensor>`同标签的另一个`<sensor>`标签下有`<camera>`标签：
```xml
<sensor name="camera" type="camera">
    <pose>0.2 0 0.2 0 0 0</pose> <update_rate>30</update_rate>
    <frame_id>laser_link</frame_id>
    <gz_frame_id>laser_link</gz_frame_id>
    <visualize>true</visualize>
    <topic>camera/image_raw</topic> 
    <camera>
        <horizontal_fov>1.089</horizontal_fov>
        <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
        </image>
        <clip>
            <near>0.05</near>
            <far>8.0</far>
        </clip>
    </camera>
</sensor>
```
同时在py的桥接器节点中添加图像和摄像机信息的传输：
```py
...
'/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
'/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
...
```
最后放入两个障碍物并将rviz2中的`/scan`话题下的`LaserScan`添加，再将这个下的`Topic`下的`Reliability Policy`改为`Best Effort`，就能看到扫描的红色轨迹，之后添加`/camera/image_raw`话题下`Image`添加，就能在左下角看到捕捉的图了，效果大概是这样：
![alt text](Image//image-4.png)
同时如果场景选择的是`sensors.sdf`，能在gazebo里看到
![alt text](Image//image-5.png)

#### 配置$\mathbf{Footprint}$
`footprint`和我们之前定义的`base_footprint`不一样，`footprint`是机器人在底面的投影，说白了就是碰撞面积，用于给规划器规划路径作为参考数据，而`base_footprint`是机器人中心在底面的投影
`footprint`更表示多边形，如果你的机器人不需要很精确的碰撞，改用`robot_radius`可直接使用半径表示碰撞面积
我们需要一个`nav2_params.yaml`文件来配置相关信息，https://github.com/ros-navigation/navigation2_tutorials/blob/rolling/sam_bot_description/config/nav2_params.yaml 是一个官方提供的nav2教程包里的默认nav2 yaml文件（最新版本下），按理来说一般不通用，因为我们没有按照教程包编写机器人，但是这个确实能让我们的代码跑起来，我们将`robot_radius`那一栏（一共有两处，全局代价地图和局部代价地图，全局代价地图上的可以不改）改为`footprint`，输入格式为
```py
footprint: "[ [0.2, 0.15], [0.2, -0.15], [-0.2, -0.15], [-0.2, 0.15] ]"
```
类似这样按顺时针或逆时针的方式输入所有角点即可，`robot_radius`可填$0.25$
我们编译运行gazebo后，运行
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```
来发送一个`map` -> `odom`的转换
随后用
```bash
ros2 launch nav2_bringup navigation_launch.py params_file:=$HOME/nav2_test/src/my_nav2_robot/config/nav2_params.yaml 
```
运行`nav2_params.yaml`文件，按理来说不出现各种`[ERROR]`和`[FATAL]`信息，再在rviz中添加两个`Polygon`，应该能看到如下场景（记住`Fixed Frame`要设置为`map`）
![alt text](Image//image-6.png)
（注意：在挪用别人的`nav2_params.yaml`时，需要注意修改`robot_base_frame`、`odom_frame`、`global_frame`（一般是`map`）和urdf里一致，同时`observation_sources`下的`topic`要和你的扫描话题`/scan`一致，最后就是其他机器人的动力学约束，和你本身的机器人一致，最重要的是$\mathbf{MPPI/DWB}$控制器参数和激光雷达的`max_obstacle_height`）