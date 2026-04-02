## $\text{Error-State Kalman Fillter}$
ESKF是EKF的升级版，它主要有**融合IMU与轮速的数据得到轮式里程计**等用途，和EKF的区别是，EKF总是需要在上一时刻后验数据附近进行线性化，这会导致精度的损失，同时需要计算雅克比矩阵，很麻烦，而ESKF则关注于误差，误差很微小，所以在误差附近所有东西都可以看作在向量空间内，进行线性化很简单

### 推导
我们定义三个量：
- 真实值：现实世界最真实的数值，由于噪声的存在无法获取，只能近似
- 名义值：不考虑噪声的理想值，非线性
- 误差量：由于噪声、飘逸等造成的偏差值，很微小，可看作线性

我们定义真实状态$\boldsymbol{x}_t$，利用算子$\oplus$（这个算子对于位置是加法，对于姿态即四元数是旋转合成，即乘法）将真实状态拆分为名义状态和误差状态：
$$\boldsymbol{x}_t = \boldsymbol{x} \oplus \delta\boldsymbol{x}$$
对应到位置、速度、姿态可得：
$$\boldsymbol{p}_t = \boldsymbol{p} + \delta \boldsymbol{p}, \quad \boldsymbol{v}_t = \boldsymbol{v} + \delta \boldsymbol{v}, \quad \boldsymbol{q}_t = \boldsymbol{q} \otimes \delta \boldsymbol{q}$$
对于imu测得的加速度$\boldsymbol{a}_m$和角速度$\omega_m$，名义状态的转移方程为：
$$\dot{\boldsymbol{p}} = \boldsymbol{v} \\
  \dot{\boldsymbol{v}} = \bold{R}(\boldsymbol{q})(\boldsymbol{a}_m - \boldsymbol{b}_a) + \boldsymbol{g} \\
  \dot{\boldsymbol{q}} = \cfrac{1}{2}\boldsymbol{q} \otimes (\omega_m - \boldsymbol{b}_g)
$$
其中$\boldsymbol{b}_a$和$\boldsymbol{b}_g$分别是**加速计零偏**和**陀螺仪零偏**，它们imu自带误差，需要自行测量抵消，但由于零偏会随imu发热变化，也就是随时间变化，导致难以测量，所以用ESKF的原因之一也是其可以动态调整零偏
而$\bold{R}(\boldsymbol{q})$是旋转矩阵，因为imu测量的加速度是机体坐标系的，而我们需要在世界坐标系进行计算，所以需要旋转矩阵转换坐标系，这个矩阵就是内层四元数的旋转矩阵
随后，我们关注误差随时间的变化，所以对其进行一阶泰勒展开：
$$\dot{\delta\boldsymbol{x}} = \bold{F}\delta\boldsymbol{x}+\bold{G}\bold{w}$$
这里的$\bold{w,F,G}$分别是**高斯白噪声**、**系统矩阵**和**噪声输入矩阵**，后者两个由对速度转移方程进行一阶泰勒展开后带入各状态量在真实值上的分解公式，通过各种推导得到：
$$\dot{\delta v} \approx -R [a_m - b_a]_\times \delta \theta - R \delta b_a + \delta g - R n_a$$
如果状态量$\delta x = [\delta p, \delta v, \delta \theta, \delta b_a, \delta b_g]$，那么$\bold{F}= \begin{bmatrix} 
0 & \mathbf{I} & 0 & 0 & 0 \\
0 & 0 & -R [a_{clean}]_\times & -R & 0 \\
0 & 0 & -[\omega_{clean}]_\times & 0 & -\mathbf{I} \\
0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0
\end{bmatrix}$，而$\bold{G}$则定义为$\begin{bmatrix} 
0 & 0 \\
-R & 0 \\
0 & -\mathbf{I} \\
0 & 0 \\
0 & 0
\end{bmatrix}$，其负责将imu的白噪声$\bold{n}_a,\bold{n}_g$带入系统
随后关注我们EKF里推导的后验估计误差协方差矩阵的转移公式：
$$\boldsymbol{P}^-_k = \Phi\boldsymbol{P}_{k - 1}\Phi^T + \boldsymbol{Q}_d$$
其中$\Phi \approx \boldsymbol{I} + \bold{F}\Delta t, \boldsymbol{Q}_d = \bold{G}\bold{Q}\bold{G}^T\Delta t$
我们设定**名义观测方程**为$h(\boldsymbol{x}_t)$，负责将名义状态从世界坐标系转到机体坐标系，对误差状态求导可得雅可比矩阵$\bold{H} = \cfrac{\partial h}{\partial \delta \boldsymbol{x}}$，类似的，我们可以套用与KF相同的卡尔曼增益公式和后验状态估计公式：
$$\boldsymbol{K}_k = \cfrac{\boldsymbol{P}^-_k\bold{H}^T}{\bold{H}\boldsymbol{P}^-_k\bold{H}^T + \bold{V}} \\
    \delta \boldsymbol{x} = \boldsymbol{K}_k(\boldsymbol{y} - h(\boldsymbol{x}))
$$
其中$\bold{V}$是观测噪声协方差矩阵，$\boldsymbol{y}$是实际观测值
最后一步实际上应该融合进后验状态估计公式中：
$$\boldsymbol{x}_{\text{new}} = \boldsymbol{x} \oplus \delta \boldsymbol{x}$$
随后更新后验估计误差协方差矩阵：
$$\boldsymbol{P}_{\text{new}} = (\boldsymbol{I} - \boldsymbol{K}_k\bold{H})\boldsymbol{P}$$
但是在此之后，由于要重置误差，我们需要执行如下两步：
$$ \boldsymbol{P}_{\text{final}} = \bold{G}_{\text{Reset}}\boldsymbol{P}_{\text{new}}\bold{G}^T_{\text{Reset}} \\
\delta\boldsymbol{x}\rightarrow 0$$
其中$\bold{G}_{\text{Reset}}$是重置雅可比矩阵，其为$\cfrac{\partial \delta \mathbf{x}_{after}}{\partial \delta \mathbf{x}_{before}}$，这一步转换其实不是必要的，可以先将其视为单位阵