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
