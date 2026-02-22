## $\mathbf{MPC}-模型预测控制$
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
$$这种二次型式子的最小值，当$\boldsymbol{Q}$为对角矩阵时，上述矩阵经过合理的换元变形能化为$\boldsymbol{A}^T\boldsymbol{A\hat{x}}=\boldsymbol{A}^T\boldsymbol{b}$型，这就是一个最小二乘法解决的**最小二乘问题**，我们从头推导这个公式
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