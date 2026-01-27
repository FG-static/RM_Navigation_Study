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
#### 生命周期