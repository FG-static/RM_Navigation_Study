# BIEVR-LIO 工作流程学习笔记

## 1. 架构定位

BIEVR-LIO 是一个在线运行的松耦合 LiDAR-Inertial Odometry（LIO）系统。它并不是先让 IMU 独立传播 10 秒，也不是在积累一段数据后做离线重定位。系统对每一帧 LiDAR 扫描依次完成 IMU 传播、点云去畸变、地图感知采样、scan-to-map 配准和地图更新，随后利用最近一段时间内已经得到的 LiDAR 位姿反向校准惯性状态。

其核心闭环可以概括为：

\[
\boxed{
\begin{aligned}
&\text{帧间 IMU 预积分与点云去畸变}\\
&\quad\rightarrow \text{BIEVR map-informed sampling}\\
&\quad\rightarrow \text{LiDAR-only scan-to-map 求当前位姿}\\
&\quad\rightarrow \text{使用最终位姿更新地图}\\
&\quad\rightarrow \text{固定最近 10 s 的 LiDAR 位姿，优化速度、bias 与重力}\\
&\quad\rightarrow \text{将最新惯性状态用于下一帧}
\end{aligned}}
\]

BIEVR-LIO 中 LiDAR 与 IMU 的职责边界非常明确：

- LiDAR scan-to-map 决定每一帧最终采用的位置和姿态；
- IMU 为当前帧 scan-to-map 提供位姿初值；
- IMU 提供扫描期间的连续运动，用于逐点去畸变；
- 最近 10 s 的 LiDAR 位姿又作为参考，帮助估计速度、加速度计零偏、陀螺仪零偏和重力方向；
- scan-to-map 的 LM 目标函数中没有 IMU residual，因此它属于松耦合，而不是 FAST-LIO 一类在同一滤波状态中直接融合 LiDAR 几何观测与 IMU 状态的紧耦合方法。

这里的“LiDAR 位姿”不是完全不使用 IMU 的意思。当前点云已经由 IMU 去畸变，配准初值也由 IMU 给出；“LiDAR-only”特指最终 scan-to-map 优化的残差来自 BIEVR 地图几何，没有把 IMU factor 同时放入该 LM 问题。

## 2. 坐标系与状态定义

本文采用如下记号：

- \(L\)：LiDAR 坐标系；
- \(I\)：IMU 坐标系，也是算法使用的 body frame；
- \(W\)：世界坐标系，即当前默认配置中的 `odom`；
- \(T_{AB}\)：将 \(B\) 系中的点变换到 \(A\) 系；
- \(T_{WI,k}\)：第 \(k\) 帧末尾 IMU 坐标系到世界坐标系的变换。

因此：

\[
p^W=T_{WI,k}p^{I_k}
=R_{WI,k}p^{I_k}+p_{WI,k}
\]

算法在 LiDAR 帧时刻保存的主要状态为：

\[
x_k=\left(R_k,p_k,v_k,b_a,b_g,g\right)
\]

其中：

- \(R_k,p_k\) 是该帧 scan-to-map 的最终结果；
- \(v_k\) 是世界坐标系下的速度；
- \(b_a,b_g\) 分别是加速度计和陀螺仪零偏；
- \(g\) 表示重力方向，代码中再乘标准重力大小 \(9.80665\,\mathrm{m/s^2}\) 得到重力向量 \(G\)。

需要特别区分两个当前帧位姿：

\[
T_{WI,k}^{pred}
\quad\text{和}\quad
T_{WI,k}^{LiDAR}
\]

前者是 IMU 从上一帧传播得到的预测，只是当前 scan-to-map 的先验；后者是 scan-to-map 利用地图几何优化后的最终位姿。下一帧传播的位姿起点采用后者。

## 3. 启动阶段不是“先传播 10 秒”

默认参数中有两个容易混淆的时间：

```yaml
imu:
  window_s: 10
  t_init: 0.2
```

它们的含义分别是：

- `t_init: 0.2`：启动时基于短时间静止假设估计初始零偏和姿态；
- `window_s: 10`：正常运行后惯性优化保留的历史窗口长度。

初始化完成后，系统先用 IMU 预测位姿把去畸变点云加入地图。当地图达到运行阈值后，系统进入正常的逐帧 scan-to-map 阶段。因此它从启动开始就是在线构图与里程计，不存在“先积累 10 秒，再统一定位”的处理。

## 4. 一帧数据的完整处理时序

设上一帧 LiDAR 结束时刻为 \(t_{k-1}\)，当前帧结束时刻为 \(t_k\)。开始处理第 \(k\) 帧时，系统已经持有：

\[
T_{WI,k-1}^{LiDAR},\quad
v_{k-1},\quad
b_a,\quad b_g,\quad G
\]

其中位姿来自上一帧 LiDAR scan-to-map，其他惯性量来自上一轮滑窗优化。当前一帧的处理顺序如下：

```text
上一帧 LiDAR 最终位姿 + 上一轮优化的惯性状态
                         │
                         ▼
             积分 t(k-1) 到 t(k) 的 IMU
                         │
              ┌──────────┴──────────┐
              ▼                     ▼
     生成整段连续运动轨迹      传播出当前位姿初值
       用于逐点去畸变          T_WI,k^pred
              │                     │
              └──────────┬──────────┘
                         ▼
       去畸变点云先做 0.1 m voxel downsample
                         │
                         ▼
       用 T_WI,k^pred 临时投影到地图并查询 voxel 分数
                         │
                         ▼
               选择高信息与粗采样点
                         │
                         ▼
        以 T_WI,k^pred 为初值进行 LM scan-to-map
                         │
                         ▼
                 T_WI,k^LiDAR
              ┌──────────┴──────────┐
              ▼                     ▼
   完整去畸变点云变换到 W       加入 10 s 状态窗口
      并融合进 BIEVR 地图        构造所有相邻 IMU factor
                                      │
                                      ▼
                           固定 LiDAR 位姿，优化惯性量
                                      │
                                      ▼
                            为第 k+1 帧提供最新状态
```

### 4.1 LiDAR 点先转换到 IMU 坐标系

系统先做距离过滤，然后使用标定外参 \(T_{IL}\) 将点从 LiDAR 坐标系转换到 IMU 坐标系：

\[
p^I=T_{IL}p^L
\]

点的相对时间和强度不会因该空间变换而改变，并继续和点坐标按索引对应。

### 4.2 IMU 预积分与当前状态预测

对 \([t_{k-1},t_k]\) 内的 IMU 数据去除当前估计的零偏并进行预积分，可得到：

\[
\Delta R_{k-1,k},\qquad
\Delta v_{k-1,k},\qquad
\Delta p_{k-1,k}
\]

从上一帧状态传播到当前帧的公式为：

\[
\boxed{R_k^{pred}=R_{k-1}^{LiDAR}\Delta R_{k-1,k}}
\]

\[
\boxed{
v_k^{pred}
=v_{k-1}+R_{k-1}^{LiDAR}\Delta v_{k-1,k}-G\Delta t
}
\]

\[
\boxed{
p_k^{pred}
=p_{k-1}^{LiDAR}
+R_{k-1}^{LiDAR}\Delta p_{k-1,k}
+v_{k-1}\Delta t
-\frac{1}{2}G\Delta t^2
}
\]

于是：

\[
T_{WI,k}^{pred}
=
\begin{bmatrix}
R_k^{pred} & p_k^{pred}\\
0 & 1
\end{bmatrix}
\]

这不是“IMU 坐标系下的位姿”，而是当前时刻 IMU frame 到世界坐标系的预测变换。IMU 的增量是在局部运动关系中形成的，但传播结果 \(R_k^{pred},p_k^{pred},v_k^{pred}\) 属于世界状态。

### 4.3 利用帧内 IMU 轨迹逐点去畸变

一帧 LiDAR 扫描中的点并非同时采集。对采样时刻为 \(t\) 的点，系统分别预测 \(t\) 和该帧最后一个点时刻 \(t_k\) 的 IMU 位姿，然后构造相对变换：

\[
T_{I_k I(t)}
=
\left(T_{WI,k}^{pred}\right)^{-1}T_{WI(t)}^{pred}
\]

再将每个点补偿到帧末 IMU 坐标系：

\[
p^{I_k}=T_{I_k I(t)}p^{I(t)}
\]

整个点云最终成为：

\[
\boxed{P_k^{I_k}}
\]

即所有点都已对齐到当前扫描最后一个点时刻的 IMU frame。源码 `undistort.cpp` 中的部分局部变量仍沿用 `L1` 命名，但当前管线传入该函数的点已经通过 \(T_{IL}\) 转到了 IMU frame，理解数据流时应以实际输入坐标系为准。

## 5. Map-informed sampling 的作用

Map-informed sampling 不是 scan-to-map 配准，而是利用当前地图判断哪些区域的点更值得进入随后的配准优化。

### 5.1 先做基础降采样

去畸变点云先按默认 \(0.1\,\mathrm{m}\) 分辨率做 voxel downsample：

\[
P_k^{I_k}
\longrightarrow
P_{k,down}^{I_k}
\]

后面的地图感知采样是在这份降采样点云上进行的，并不是直接处理最原始的全分辨率点云。

### 5.2 预测位姿只用于地图查询

对降采样后的每个点 \(p_i^{I_k}\)，系统临时用预测位姿投到世界坐标系：

\[
\hat p_i^W=T_{WI,k}^{pred}p_i^{I_k}
\]

根据 \(\hat p_i^W\) 查询它大致落入的 BIEVR voxel，并读取该 voxel 的 `mean_img_dist` 信息分数。预测位姿此时只用于回答“这个点大概对应地图中的哪个 voxel”，不会把待配准 source point 永久保存成世界坐标。

采样后的点仍然表示为：

\[
P_{k,selected}^{I_k}
\]

而不是 \(P_{k,selected}^{W}\)。因此不存在“先把点变到 world，再从原始点云重新找回来做配准”的额外坐标往返；实现中通过索引直接从 \(P_{k,down}^{I_k}\) 取出相应点。

### 5.3 高信息区域与低信息区域采用不同密度

系统按照 voxel 分数从高到低排序，并选择一定数量的高信息 voxel：

- 对高信息 voxel，保留该 voxel 中所有已经过 0.1 m 降采样的点，形成 fine points；
- 对其余 voxel，每个 voxel 仅保留一个代表点，形成 coarse points；
- fine points 与 coarse points 合并后作为 scan-to-map 的 source cloud。

墙缝、凹槽、轨枕、石块、管线等具有明显局部起伏的区域通常比大面积平滑墙面提供更强的配准信息。该策略把更多计算量分配给几何区分度高的区域，同时仍用粗采样点维持整体覆盖。

## 6. LM scan-to-map 如何得到最终位姿

完成采样后，配准问题包含：

- source：\(P_{k,selected}^{I_k}\)；
- target：已经在线建立的 BIEVR world map；
- initial guess：\(T_{WI,k}^{pred}\)。

BIEVR voxel 会在其局部坐标系中保存表面高度图。将 source point 变换到对应 voxel 的局部坐标系后，点的坐标记为 \(p_o=(x,y,z)\)，高度图在 \((x,y)\) 处的插值为 \(I(x,y)\)，几何残差可写为：

\[
r=z-I(x,y)
\]

LM 迭代在 \(SE(3)\) 上求取增量：

\[
T\leftarrow T\operatorname{Exp}(\delta\xi)
\]

并从预测初值逐步优化得到：

\[
\boxed{
T_{WI,k}^{LiDAR}
=\operatorname{ScanToMap}
\left(P_{k,selected}^{I_k},T_{WI,k}^{pred},\mathcal M_{k-1}\right)
}
\]

也可以从概念上写成：

\[
T_{WI,k}^{LiDAR}
=T_{WI,k}^{pred}\Delta T_{LiDAR}
\]

但 \(\Delta T_{LiDAR}\) 应理解为 LM 多次迭代累积得到的地图几何修正，而不是一个预先独立计算出来、再与 IMU pose 做坐标变换的量。配准结束后，系统采用 \(T_{WI,k}^{LiDAR}\) 作为当前帧正式位姿，原来的 \(T_{WI,k}^{pred}\) 不再作为最终输出。

## 7. 为什么用完整点云更新地图

scan-to-map 为控制计算量只使用经过 informed sampling 的点，但位姿确定以后，系统会回到完整的去畸变点云：

\[
P_k^W=T_{WI,k}^{LiDAR}P_k^{I_k}
\]

然后将 \(P_k^W\) 融合进 BIEVR map，更新 voxel 的局部坐标、法向、bump image、像素权重和信息分数等地图内容。

因此应区分：

- `source_filtered`：用于计算当前位姿；
- `points_undistorted_I`：当前帧的完整去畸变点云；
- `points_registered`：用最终 LiDAR 位姿变换到世界坐标系、用于地图融合和发布的完整点云。

## 8. 最近 10 秒惯性滑窗

### 8.1 每帧重新构造优化问题

惯性窗口不是一个永远在后台持续运行的 Ceres optimizer。每处理完一帧，系统都会新建一个 `ceres::Problem`，将当前时间窗口内保存的 IMU 预积分段重新加入 residual blocks，然后求解一次。下一帧到来时再用滑动后的数据重新建题、重新求解。

若当前窗口中的 LiDAR 状态为：

\[
x_0,x_1,\ldots,x_N
\]

相邻状态间的 IMU 预积分段为：

\[
\mathcal I_{01},\mathcal I_{12},\ldots,\mathcal I_{N-1,N}
\]

则 Ceres 中加入的是所有相邻状态对应的 factor：

```text
T0        T1        T2                  TN
●---------●---------●-------------------●
    I01       I12            ... I(N-1,N)
```

整体目标是：

\[
\boxed{
\min
\sum_{i=0}^{N-1}
\left\|r_{i,i+1}\right\|^2
+\left\|r_g\right\|^2
}
\]

而不是只建立一个从 \(T_0\) 到 \(T_N\) 的 factor，也不是只要求 IMU 从 10 秒前的状态最终落到当前 LiDAR pose。

### 8.2 单个 IMU factor 的约束

对两个相邻的、由 LiDAR scan-to-map 得到的位姿：

\[
T_i=(R_i,p_i),\qquad T_j=(R_j,p_j)
\]

预积分给出经过 bias 修正的 \(\Delta R_{ij}\)、\(\Delta p_{ij}\) 和 \(\Delta v_{ij}\)。代码中的 9 维 residual 可以概括为：

\[
r_R
=2\operatorname{vec}\!\left[
\Delta q_{ij}^{-1}
\left(q_i^{-1}q_j\right)
\right]
\]

\[
r_p
=R_i^T
\left(
\frac{1}{2}G\Delta t^2+p_j-p_i-v_i\Delta t
\right)
-\Delta p_{ij}
\]

\[
r_v
=R_i^T
\left(G\Delta t+v_j-v_i\right)
-\Delta v_{ij}
\]

这些残差分别表达：

- LiDAR 观测到的相邻姿态变化应与陀螺仪预积分一致；
- LiDAR 观测到的相邻位置变化应能被速度、重力和加速度预积分解释；
- 相邻时刻的速度变化应能被重力和加速度预积分解释。

预积分增量还会通过雅可比对 \(b_a,b_g\) 做一阶修正；当加速度计 bias 变化超过实现中的阈值时，积分器会重新积分该段 IMU 数据。

### 8.3 哪些变量固定，哪些变量被优化

窗口内所有由 LiDAR scan-to-map 得到的：

\[
R_0,p_0,R_1,p_1,\ldots,R_N,p_N
\]

全部设为常量。优化变量是：

\[
v_1,v_2,\ldots,v_N
\]

以及窗口内所有 factor 共享的：

\[
b_a,\qquad b_g,\qquad g
\]

窗口最早状态的 \(v_0\) 也会固定，用作速度锚点。重力方向被限制在单位球面上，并增加一个先验以抑制无约束漂移；重力大小固定为标准重力加速度。

这说明 10 s 滑窗不会回头修改已经用于建图的历史 LiDAR pose，也不会重建过去的地图。它是在接受 LiDAR 位姿历史的前提下，寻找一组能够最好解释这些位姿变化的速度、零偏和重力方向。

### 8.4 为什么优化历史 10 秒并非“只有最后一下有用”

直接用于下一帧传播的确主要是窗口末端状态：

\[
T_{WI,k}^{LiDAR},\quad
v_k^{opt},\quad
b_a^{opt},\quad
b_g^{opt},\quad
g^{opt}
\]

历史速度 \(v_{k-N},\ldots,v_{k-1}\) 更多是窗口中的中间变量，但它们并非无用。原因在于：

1. 每个相邻 LiDAR 位姿都为对应的短 IMU 段提供约束，而不是只看一个总起点和总终点；
2. \(b_a,b_g,g\) 是整个窗口共享的量，任何一段上的误差都会共同推动它们修正；
3. 中间速度把相邻的位置残差与速度残差连接起来，使最新 \(v_k\) 能利用整个窗口的运动历史，而非只依赖最近一小段 IMU；
4. 较长时间内多种方向和加速度的运动有助于区分真实运动、零偏和重力方向。

下一帧仍处于窗口内的历史状态会再次参与新一轮优化；超过 `window_s` 的状态和相应 IMU integrator 才会被移除。

## 9. 三个首尾相接的核心关系

BIEVR-LIO 的主循环最终可以压缩为三个关系式。

第一，使用上一帧 LiDAR 最终位姿和已优化惯性量，预测当前帧：

\[
\boxed{
T_{WI,k}^{pred}
=\operatorname{IMUPropagate}
\left(
T_{WI,k-1}^{LiDAR},v_{k-1},b_a,b_g,g,
\operatorname{IMU}_{k-1:k}
\right)
}
\]

第二，使用预测位姿作为初值，由 LiDAR 与在线地图决定最终位姿：

\[
\boxed{
T_{WI,k}^{LiDAR}
=\operatorname{ScanToMap}
\left(
P_k^{deskew},T_{WI,k}^{pred},\mathcal M_{k-1}
\right)
}
\]

第三，固定最近窗口内的 LiDAR 位姿，利用所有相邻 IMU factor 优化惯性量：

\[
\boxed{
\left(v_k,b_a,b_g,g\right)^{opt}
=\operatorname{WindowOptimize}
\left(
T_{WI,k-N:k}^{LiDAR},
\operatorname{IMU}_{k-N:k}
\right)
}
\]

这三步首尾相接，形成：

\[
\boxed{
\text{IMU 预测 LiDAR 配准初值}
\rightarrow
\text{LiDAR 地图配准修正位姿}
\rightarrow
\text{LiDAR 位姿历史校准惯性状态}
\rightarrow
\text{下一帧 IMU 预测}
}
\]

## 10. 容易混淆的表述

| 容易产生误解的说法 | 更准确的表述 |
|---|---|
| IMU 先传播 10 秒 | IMU 每个 LiDAR 帧间隔都传播；10 秒是惯性优化历史窗口 |
| 上一帧位姿是 IMU 积分得到的 | 上一帧最终 \(R,p\) 来自 LiDAR scan-to-map，IMU 只为当前帧传播初值 |
| 去畸变后仍是 LiDAR 系点云 | 点先经 \(T_{IL}\) 进入 IMU 系，再被统一补偿到帧末 \(I_k\) |
| 第四步就是 scan-to-map | 第四步是 map-informed sampling，第五步才是真正的 LM 配准 |
| 采样先把 source point 永久变到 world | \(T_{WI}^{pred}\) 只用于查询地图 voxel；选出的 source point 仍在 \(I_k\) |
| LiDAR 对 IMU pose 做一次坐标变换 | LM 从 IMU prediction 出发，使用地图几何优化出新的最终 \(T_{WI}\) |
| 10 秒内保存的是 LiDAR 预测位姿 | 保存并固定的是 LiDAR registration estimated poses |
| IMU factor 优化器持续在后台运行 | 每帧重新构造当前窗口的 Ceres 问题并求解 |
| 只需让 10 秒前的 IMU 积分对上最后一个 pose | 所有相邻 LiDAR pose 之间都有 IMU factor，最小化所有分段残差之和 |
| 优化完历史速度没有意义 | 历史速度是连接各分段约束的中间变量，并共同约束共享 bias、重力和最新速度 |
| 固定位姿说明系统是离线重定位 | 固定位姿只发生在在线惯性状态校准阶段；每帧位姿、地图和输出均在线产生 |

## 11. 源码阅读入口

本文结论对应当前工程 `/home/goose/bievr_ws/src/BIEVR-LIO` 中的以下实现：

- `BIEVR/src/pipeline.cpp`：每帧完整主流程、状态窗口维护和 Ceres 惯性优化；
- `BIEVR/src/imu_integrator.cpp`：IMU 预积分、状态传播公式和 9 维惯性残差；
- `BIEVR/src/undistort.cpp`：按每个点的时间戳补偿到帧末坐标系；
- `BIEVR/src/preprocess.cpp`：预测位姿查图、voxel 评分、fine/coarse 点选择；
- `BIEVR/src/ls_optimizer.cpp`：BIEVR 地图残差与 LM 位姿优化；
- `BIEVR/src/inertial_factor.cpp`：惯性 factor 的参数块和 Jacobian；
- `BIEVR/include/bievr_lio/pipeline.h`：状态、外参、窗口及运行配置；
- `config/params.yaml`：默认 0.1 m 降采样、10 s 惯性窗口、0.2 s 初始化时间和 `odom` 地图坐标系。

