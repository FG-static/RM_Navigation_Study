# VINS for Learning

## VIO / VINS 基本概念

### VIO
Visual-Inertial Odometry，视觉惯性里程计。

主要利用 Camera + IMU，估计：

\[
p,\ R,\ v,\ b_a,\ b_g
\]

重点是局部、连续的运动估计，本身允许长期漂移。

### VINS
Visual-Inertial Navigation System。

可以理解为：

\[
\text{VIO} + \text{初始化 / 滑窗 / 回环 / 全局优化 / 重定位}
\]

粗略地：

\[
\boxed{\text{VIO} \subseteq \text{VINS}}
\]

---

## 从 BA 理解 VINS

### BA 是什么

视觉观测：

\[
z_{ij}
\]

当前相机 Pose 和 Landmark 预测出的像素：

\[
\hat z_{ij}=\pi(T_{iw}P_j)
\]

残差：

\[
r_{visual}=z_{ij}-\hat z_{ij}
\]

BA 就是同时调整 \(T_i,P_j\)，让所有重投影误差尽量小。

核心思想：

\[
\boxed{\text{Measurement} \leftrightarrow \text{当前状态预测出的 Measurement}}
\]

### SLAM 前后端

前端负责制造约束，例如：

- 特征提取
- 特征跟踪
- 匹配
- 关键帧选择

后端负责利用这些约束优化状态。

---

## 加入 IMU 后

状态从纯视觉的 Pose 扩展为：

\[
X_i=[p_i,R_i,v_i,b_{a_i},b_{g_i}]
\]

因为 IMU 描述的是动力学：

\[
p_{k+1}=p_k+v_k\Delta t+\frac12a\Delta t^2
\]

\[
v_{k+1}=v_k+a\Delta t
\]

\[
R_{k+1}=R_k\operatorname{Exp}(\omega\Delta t)
\]

所以必须估计速度和 Bias。

---

## IMU Propagation

IMU 原始测量：

\[
\tilde\omega=\omega+b_g+n_g
\]

\[
\tilde a=R^T(a-g)+b_a+n_a
\]

给定当前状态 \(X_i\)，IMU 可以直接往前预测：

\[
R_{k+1}=R_k\operatorname{Exp}((\tilde\omega-b_g)\Delta t)
\]

\[
v_{k+1}=v_k+[R_k(\tilde a-b_a)+g]\Delta t
\]

\[
p_{k+1}=p_k+v_k\Delta t+\frac12[R_k(\tilde a-b_a)+g]\Delta t^2
\]

用途：

\[
\boxed{\text{给下一状态提供预测 / Initial Guess}}
\]

---

## IMU Preintegration

### 为什么需要

Camera 低频，IMU 高频：

```text
Frame i
  |
  | imu
  | imu
  | imu
  | ...
  |
Frame j
```

如果后端每次优化修改 \(R_i\) 后都重新积分所有 IMU，会很麻烦。

于是把中间 IMU 压缩成：

\[
\boxed{\Delta R_{ij},\ \Delta v_{ij},\ \Delta p_{ij}}
\]

### 三个量怎么理解

\(\Delta R_{ij}\)：IMU 认为从 \(i\) 到 \(j\) 相对旋转了多少。

\(\Delta v_{ij}\)：去掉重力后，在第 \(i\) 帧坐标系下积累的速度变化。

\(\Delta p_{ij}\)：去掉初速度和重力后，在第 \(i\) 帧坐标系下积累的位移。

关键关系：

\[
R_j=R_i\Delta R_{ij}
\]

\[
v_j=v_i+g\Delta t+R_i\Delta v_{ij}
\]

\[
p_j=p_i+v_i\Delta t+\frac12g\Delta t^2+R_i\Delta p_{ij}
\]

预积分量应该理解成：

\[
\boxed{\text{IMU Measurement}}
\]

不是最终状态。

---

## IMU Residual

这是最容易混的地方。

IMU 预积分给出：

\[
\Delta R_{ij}^{IMU}
\]

当前优化状态 \(R_i,R_j\) 本身隐含：

\[
R_i^TR_j
\]

所以：

\[
\boxed{
r_R=
\operatorname{Log}\left[(\Delta R_{ij}^{IMU})^{-1}R_i^TR_j\right]
}
\]

比较的是：

\[
\boxed{
\text{IMU 测出来的相对旋转}
\leftrightarrow
\text{当前优化状态隐含的相对旋转}
}
\]

不是“两个 IMU 结果互相比较”。

### 为什么一开始 residual 接近 0

因为 \(X_j\) 往往就是 IMU propagation 初始化出来的：

\[
R_j^{pred}=R_i\Delta R_{ij}
\]

所以：

\[
R_i^TR_j\approx\Delta R_{ij}
\]

自然有：

\[
r_R\approx0
\]

但进入优化以后，\(R_i,R_j\) 已经是待优化变量。

视觉约束可能把它们拉动，于是 IMU residual 开始非零，并对这种变化产生约束。

### 完整 IMU Residual

位置：

\[
r_p=
R_i^T\left(p_j-p_i-v_i\Delta t-\frac12g\Delta t^2\right)
-\Delta p_{ij}
\]

速度：

\[
r_v=R_i^T(v_j-v_i-g\Delta t)-\Delta v_{ij}
\]

旋转：

\[
r_R=\operatorname{Log}\left[(\Delta R_{ij})^{-1}R_i^TR_j\right]
\]

Bias：

\[
r_{b_a}=b_{a_j}-b_{a_i}
\]

\[
r_{b_g}=b_{g_j}-b_{g_i}
\]

---

## Bias / Jacobian / Covariance

预积分时使用某个 Bias：

\[
\bar b_a,\ \bar b_g
\]

优化后 Bias 小幅变化时，用 Jacobian 一阶修正：

\[
\Delta p\approx\bar{\Delta p}+J_{p,b_a}\delta b_a+J_{p,b_g}\delta b_g
\]

\[
\Delta v\approx\bar{\Delta v}+J_{v,b_a}\delta b_a+J_{v,b_g}\delta b_g
\]

Bias 变化太大时重新积分：

\[
\boxed{\text{repropagation}}
\]

IMU 有噪声，所以还需要传播 Covariance：

\[
P_{k+1}=F_kP_kF_k^T+G_kQG_k^T
\]

误差状态一般是：

\[
[\delta p,\delta\theta,\delta v,\delta b_a,\delta b_g]
\]

共 15 维，所以源码里经常看到 \(15\times15\) 矩阵。

---

## Sliding Window / Marginalization

VINS 不会一直保留所有历史帧，只保留有限窗口：

```text
X1 X2 X3 ... X10
```

旧状态不能直接删，因为会丢失历史信息。

所以通过 Marginalization，把旧状态的信息压成：

\[
\boxed{\text{Prior Factor}}
\]

常用 Schur Complement：

\[
H'=H_{rr}-H_{rm}H_{mm}^{-1}H_{mr}
\]

即：旧变量消失，但旧信息继续留在窗口里。

---

## 和 LIO Pose Graph 的类比

LIO 中 Odom Edge：

\[
Z_{ij}^{odom}
\]

当前节点状态隐含：

\[
X_i^{-1}X_j
\]

残差：

\[
r_{odom}=\operatorname{Log}\left[(Z_{ij}^{odom})^{-1}X_i^{-1}X_j\right]
\]

刚开始节点就是靠 odom 累计出来的，所以：

\[
r_{odom}\approx0
\]

加入 Loop Edge 后，回环残差较大，优化器开始移动节点。

节点一动，原本接近 0 的 Odom Edge 也会产生残差。

最终不是单独让 Loop Residual 为 0，而是：

\[
\boxed{\min_X\sum_i r_i^T\Omega_i r_i}
\]

即所有 Edge / Factor 的加权残差平方和整体最小。

和 VINS 的对应：

\[
\Delta R_{ij}^{IMU}
\leftrightarrow
R_i^TR_j
\]

对应 LIO：

\[
Z_{ij}^{odom}
\leftrightarrow
X_i^{-1}X_j
\]

---

## 一句话总结

VINS 不是：

```text
视觉算一个 Pose
IMU 算一个 Pose
然后两者加权平均
```

而是：

```text
视觉观测 → Visual Factor
IMU       → IMU Factor
历史信息 → Prior Factor
                ↓
             Optimizer
                ↓
        p, R, v, ba, bg
```

核心始终是：

\[
\boxed{
\text{Measurement}
\leftrightarrow
\text{当前状态预测出的 Measurement}
}
\]

优化器寻找一组状态，使所有 Factor 的加权残差平方和尽量小。
