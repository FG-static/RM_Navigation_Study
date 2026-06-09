# $$\text{Control Algorithm for Learning}$$
## $\text{MPC}$
### 基本概念
#### 最优控制
最优控制的研究理念是在**约束条件**下达到**最优**的系统表现，以数学方式来描述这个最优化问题：
$r(t)-\mathrm{Reference}$到$u(t)-\mathrm{Input}$到$y(t)-\mathrm{Output}$再回到$r(t)$，是一个**单输入单输出系统**$\mathrm{(SISD)}$，定义误差$e=y-r$，从轨迹追踪的角度来说：
- $\int^t_0 e^2 dt$越小，误差越小，追踪越好
- $\int^y_0 u^2 dt$越小，输入越小，系统越容易以更少的能量稳定

我们可以去设定一个目标代价函数
$$J = \int^t_0 (qe^2 + ru^2) dt
$$其中$q$和$r$分别是**误差代价权重**和**控制代价权重**，通过设置一个控制器$u$去调节这两个参数使得$J$最小，如果$q$设计的远大于$r$，表示我们更看重误差，如果$r$设计的远大于$q$，表示我们更看重输入
而在**多输入多数出系统**$(\mathrm{MIMD})$中，对于状态空间：
$$\begin{cases}
    \cfrac{d\boldsymbol{x}}{dt} = \boldsymbol{Ax} + \boldsymbol{Bu} \\
    \boldsymbol{y} = \boldsymbol{Cx}
\end{cases}
$$其中$\boldsymbol{x},\boldsymbol{y}$分别是状态变量和输出，于是可以设计它的目标代价函数：
$$J = \int^{t}_0 \boldsymbol{e}^T \boldsymbol{Qe} + \boldsymbol{u}^T\boldsymbol{Ru}
$$其中矩阵$\boldsymbol{Q,R}$是**调节矩阵**，都是对角矩阵，矩阵元素则是权重系数，用来决定更关注哪些状态量，更关注哪些输入（值的注意的是，MPC等其他控制算法在描述公式时基本上都会出现三明治类型的项，例如$\boldsymbol{e}^T\boldsymbol{Qe}$，它们被称为二次型，同时内部矩阵$\boldsymbol{Q}$基本被要求为是**半正定**或**正定**的，这保证了这个式子总会有最小值）

#### $\mathrm{MPC}$
对于mpc来说，它多用于数位控制和离散型状态空间表达，它主要有三个步骤：
在$k$时刻
- 估计或测量读取当前的系统状态
- 基于$u_k,u_{k+1},\cdots,u_{k+N}$来进行最优化，我们设定一个目标代价函数
  $$J = \sum\limits_k^{N - 1} (\boldsymbol{e}^T_k\boldsymbol{Qe}_k + \boldsymbol{u}^T_k\boldsymbol{Ru}_k + \boldsymbol{e}^T_N\boldsymbol{Fe}_N)
  $$其中第三项是**终端误差代价**，$\boldsymbol{F}$是**终端代价权重矩阵**，随后对整个目标代价函数进行最优化，同时在求解这个问题时还需要附上系统的约束
  数据获取过程具体来说就是设定一个**控制区间**，在这个区间内去预测例如$u_k,u_{k+1},u_{k+2}$的输入的结果$y_{k+1},y_{k+2},y_{k+3}$，这些输入的输出所在区间为**预测区间**
- 最后，我们只选取上一步预测的$u_k$作为$k$时刻的输入，而舍弃后面时刻，随后我们均将两个区间向右移动一时刻，继续上述步骤，这就叫**滚动优化控制**$(\mathbf{Receding~Horizon~Control})$

##### 二次规划$(\text{QP})$
二次规划的一般形式是求解
$$
\boldsymbol{z}^T\boldsymbol{Qz}+\boldsymbol{C}^T\boldsymbol{z}
$$这种二次型式子的最小值，当$\boldsymbol{Q}$为对角矩阵时，上述矩阵经过合理的换元变形能化为$\boldsymbol{A}^T\boldsymbol{A\hat{x}}=\boldsymbol{A}^T\boldsymbol{b}$型，这就是一个最小二乘法解决的**最小二乘问题**$(\text{LS})$，最小二乘问题形式上和QP是一样的，不同在于LS通常无约束，同时后面推导的$\boldsymbol{H}$必须是正定或半正定的，而QP没有要求，我们从头推导这个公式
我们规定$u_{i|j}$表示在$j$时刻预测时预测到的$i$时刻时的输入，同理定义$x_{i|j}$
在$k$时刻，我们有一个预测区间$N$，我们需要得到：
$$
\boldsymbol{X}_k = \begin{bmatrix}
  x_{k|k} \\
  x_{k + 1|k} \\
  x_{k + 2|k} \\
  \vdots \\
  x_{k + N|k}
\end{bmatrix},\boldsymbol{U}_k = \begin{bmatrix}
  u_{k|k} \\
  u_{k + 1|k} \\
  u_{k + 2|k} \\
  \vdots \\
  u_{k + N - 1|k}
\end{bmatrix}
$$即状态量向量和输入量向量，此时我们令参考值为$0$，$\boldsymbol{C} = \boldsymbol{I}$，那么有输出等于状态，此时的目标代价函数就是：
$$
J = \sum\limits_{i = 0}^{N - 1} (x_{k + i|k}^TQx_{k + i|k} + u_{k + i|k}^TRu_{k + i|k}) + x_{k + N|k}^TFx_{k + N|k}
$$我们一开始有初始条件$x_k = x_{k|k}$，问题在于怎么把在$k$时刻预测其他时刻得到的状态量给全部化为由$u$和$x_k$表示，实际上只要根据$x_{k+1|k} = \boldsymbol{A}x_{k|k}+\boldsymbol{B}u_{k|k}$这个公式不断代换递推就能得到
$$
x_{k+N|k} = \boldsymbol{A}^Nx_k + \sum\limits_{i = 0}^{N - 1} \boldsymbol{A}^{N - 1 - i}\boldsymbol{B}u_{k + i|k}
$$通过化简，能得到
$$
\boldsymbol{X}_k = \begin{bmatrix}
  \boldsymbol{I} \\
  \boldsymbol{A} \\
  \boldsymbol{A^2} \\
  \vdots \\
  \boldsymbol{A}^N
\end{bmatrix}x_k + \begin{bmatrix}
  0 & 0 & \cdots & 0 \\
  \boldsymbol{B} & 0 & \cdots & 0 \\
  \boldsymbol{AB} & \boldsymbol{B} & \cdots & 0 \\
  \vdots & \vdots & & 0 \\
  \boldsymbol{A}^{N - 1}\boldsymbol{B} & \boldsymbol{A}^{N - 2}\boldsymbol{B} & \cdots & \boldsymbol{B}
\end{bmatrix}\begin{bmatrix}
  u_{k|k} \\
  u_{k + 1|k} \\
  \vdots \\
  u_{k + N - 1|k}
\end{bmatrix}
$$令第一项和第二项的左乘矩阵分别为$\boldsymbol{M, C}$，那么就有
$$
\boldsymbol{X}_k = \boldsymbol{M}x_k + \boldsymbol{CU}_k
$$同时对$J$展开，我们也能得到
$$
J = \boldsymbol{X}^T_k\bar{\boldsymbol{Q}}\boldsymbol{X}_k + \boldsymbol{U}^T_k\bar{\boldsymbol{R}}\boldsymbol{U}_k
$$其中$\bar{\boldsymbol{Q}} = \begin{bmatrix}
  \boldsymbol{Q} & 0 & \cdots & 0 \\
  0 & \boldsymbol{Q} & \cdots & 0 \\
  \vdots & \vdots & & \vdots \\
  0 & 0 & \cdots & \boldsymbol{F}
\end{bmatrix}$，而$\bar{\boldsymbol{R}}$则是一个全为$\boldsymbol{R}$的对角矩阵
再将上述式子带入$J$，再经过化简整理能得到
$$
J = x^T_k\boldsymbol{G}x_k + 2x^T_k\boldsymbol{E}\boldsymbol{U}_k + \boldsymbol{U}_k\boldsymbol{HU}_k
$$其中$\boldsymbol{G,E,H}$分别是$\boldsymbol{M}^T\bar{\boldsymbol{Q}}\boldsymbol{M},\boldsymbol{C}^T\bar{\boldsymbol{Q}}\boldsymbol{M},\boldsymbol{C}^T\bar{\boldsymbol{R}}\boldsymbol{C} + \boldsymbol{R}$
这个形式实际上是一个初始状态 + 一个线性型 + 一个二次型，和最开始的式子基本一致
如果引入了$x_{\mathrm{ref}}$，即路径采样点（根据每$\Delta t$的步长采取的规划器计算出来的路径点），类似$x_k$的定义以定义$\boldsymbol{X}_{k,\mathrm{ref}}$
那么代价函数变为
$$
J = (\boldsymbol{X}_k - \boldsymbol{X}_{ref})^T \bar{\boldsymbol{Q}} (\boldsymbol{X}_k - \boldsymbol{X}_{ref}) + \boldsymbol{U}_k^T \bar{\boldsymbol{R}} \boldsymbol{U}_k
$$
此时$\boldsymbol{H}$的形式不变，需要重新定义$\boldsymbol{f} = \boldsymbol{W} = \boldsymbol{C}^T \bar{\boldsymbol{Q}} (\boldsymbol{M}x_k - \boldsymbol{X}_{\text{ref}}),\boldsymbol{G} = (\boldsymbol{Mx}_k - \boldsymbol{X}_{ref})^T \bar{\boldsymbol{Q}} (\boldsymbol{Mx}_k - \boldsymbol{X}_{ref})$
此时式子变为：
$$
J = \boldsymbol{G} + 2\boldsymbol{f} \boldsymbol{U}_k + \boldsymbol{U}_k^T \boldsymbol{H} \boldsymbol{U}_k
$$常数项（即$\boldsymbol{G}$）一般不影响最终求解结果，所以我们只关注后两项，和上述形式是一样的
于是最后我们只需要调整适当的$\boldsymbol{Q,R,F}$参数即可
- 1. $\boldsymbol{Q}$调大，更信任路径；调小，允许有偏离路径的行为
- 2. $\boldsymbol{R}$调大，通过惩罚减少变化的增量大小；调小，完全信任电机响应能力，反应迅速灵敏
- 3. $\boldsymbol{F}$调大，可能在终点前迅速大幅度偏转变速回正；调小，可能无法达到理想的终点位姿

## $\text{PID}$
### $\text{拉普拉斯变换 - Laplace Transform}$
我们称使得复数$s$经过函数$F(s)$得到：
$$
F(s) = \int^{\infty}_0 f(t)e^{-st} dt
$$
的$F$称为**拉普拉斯变换**

#### $\text{3Blue1Brown视频笔记}$
参考视频：
- [But what is a Laplace Transform?](https://www.youtube.com/watch?v=j0wJBEZdwLs)
- [Why Laplace transforms are so useful](https://www.youtube.com/watch?v=FE-hM1kRK4Y)

拉普拉斯变换可以理解为：把一个随时间变化的信号$f(t)$，拿去和一族指数函数$e^{-st}$做加权积分，从而观察这个信号在不同复数参数$s$下的整体响应。这里的$s$不是普通的时间变量，而是一个复数：
$$
s = \sigma + i\omega
$$
因此
$$
e^{-st}=e^{-\sigma t}e^{-i\omega t}
$$
其中$e^{-\sigma t}$控制信号随时间衰减或增长的权重，$e^{-i\omega t}$对应旋转的复平面相位。直观上，拉普拉斯变换是在问：如果把原信号按某个频率“绕起来”，再按某个指数衰减方式加权，它整体会偏向哪里、偏向多少？

第一集强调的是几何直觉：傅里叶变换主要关注纯频率成分，而拉普拉斯变换多了一个实部$\sigma$，所以不仅能描述振荡频率，还能描述增长、衰减和稳定性。对于控制系统来说，这一点很重要，因为系统响应往往不仅有“振荡得多快”，还有“会不会衰减到稳定状态”或“会不会发散”的问题。

第二集强调的是工程用途：拉普拉斯变换能把微分方程变成代数方程。因为在零初始条件下有：
$$
\mathcal{L}\left(\cfrac{df}{dt}\right)=sF(s)
$$
以及
$$
\mathcal{L}\left(\int f(t)dt\right)=\cfrac{1}{s}F(s)
$$
于是时域中的微分、积分运算，在$s$域中会分别变成乘以$s$和除以$s$。这就是控制系统中常用传递函数的基础：原本需要求解动态微分方程的问题，可以转换成关于$s$的代数表达式来分析。

对于PID控制来说，这个观点能解释为什么：
$$
u(t)=k_pe(t)+k_I\int e(t)dt+k_D\cfrac{de(t)}{dt}
$$
经过拉普拉斯变换后会得到：
$$
U(s)=\left(k_p+\cfrac{k_I}{s}+k_Ds\right)E(s)
$$
其中比例项$k_p$保持不变，积分项变成$\cfrac{k_I}{s}$，微分项变成$k_Ds$。因此PID在$s$域中可以看成一个控制器传递函数：
$$
C(s)=k_p+\cfrac{k_I}{s}+k_Ds
$$

从控制角度看，$s$域表达不仅是换一种写法，更方便观察系统的极点、零点和稳定性。极点位置决定系统响应是否稳定、是否振荡以及收敛快慢：如果极点实部为负，响应会衰减；如果极点实部为正，响应会发散；如果靠近虚轴，系统往往衰减慢或振荡明显。因此拉普拉斯变换的核心意义是：把时域中的动态过程，转换成更容易分析稳定性、频率响应和控制器结构的$s$域模型。

### 基本概念
和mpc算法一样，pid算法也有参考$r$-输入$u$-输出-参考这一循环过程，实际上pid算法并不比mpc算法更优，不过其优点也是其实现简单，占用资源不大，它也是$\text{PI}$控制和$\text{PD}$控制的结合
对于pid算法，我们主要有三个核心要点
- **当前误差**
  当前误差由$k_pe$表示，其中$k_p$是比例增益，$e$是参考值与输出值的误差，而比例增益是用来决定我们调节的幅度的，以调节水温为例，在水凉了时越凉比例增益越大，我们要调的就越大，反之亦然。我们要通过误差来调节输入$u$
- **过去误差**
  过去误差由$k_I\int edt$表示，它会同时注重过去积累的误差，其中$k_I$是积分增益，效果与地位和$k_p$一致
- **变化趋势**
  变化增益由$k_D\cfrac{de}{dt}$表示，其中$k_D$是微分增益，和$k_p$一致，还是以水温为例，这个关心的就是水温变化的速度

将上述三项加在一起就得到了pid控制的核心公式：
$$u = k_pe + k_I\int edt + k_D\cfrac{de}{dt}$$
两边作拉普拉斯变换就能得到pid的**传递函数**（描述线性时不变系统输入输出动态关系的数学模型）：
$$\mathcal{L}(u) = \mathcal{L}(k_pe + k_I\int edt + k_D\cfrac{de}{dt}) \\
u_{(s)} = (k_p + k_I\cfrac{1}{s} + k_Ds)E_{(s)}$$
其中$s$是拉普拉斯算子
回到原公式，我们从比例增益开始，如果只有比例增益，其他均为0，那么整个系统是无法将误差降为0的（很难），最终可能会浮动到例如1上下的位置，这是因为越接近0比例增益就会越小，最终可能无法到达0，例如这个样图：
![alt text](Image//image-17.png)
现在加上积分增益，其略小于比例增益，我们能看到确实误差最后稳定在了0，但是整个变化过程很漫长，且振幅很大，这是因为参考了积分增益相当于考虑了过去的误差，它会通过误差累计来修正，但是这个修正很迟钝，由于累计误差很大，导致每次修正的修正量也很大，同时它不会去考虑修正时的变化趋势（也就是变化速度），所以导致很难快速的修正下来，例如这个样图：
![alt text](Image//image-18.png)
现在加上了微分增益，略小于积分增益，可以看到很快就平稳了，没有过大的震荡也没有漫长的修正时间，但是有一个问题就是输入初始值非常的大，这类似于重疾要用猛药，过大的输入会需要非常高的能量，这明显也是不利的，例如这个样图：
![alt text](Image//image-19.png)
如果我们在输入加上一个非常微小的变化非常迅速的白噪音，这样就会导致整个系统的输入变化非常的剧烈且大，这是微分增益导致的，因为微分会放大快速变换的地方，例如$0.001\sin 1000t$这个输入，对其微分后是$\cos1000t$，放大了$1000$倍
