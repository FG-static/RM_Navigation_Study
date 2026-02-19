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
$$其中矩阵$\boldsymbol{Q,R}$是**调节矩阵**，都是对角矩阵，矩阵元素则是权重系数，用来决定更关注哪些状态量，更关注哪些输入

#### $\mathrm{MPC}$
对于mpc来说，它多用于数位控制和离散型状态空间表达，它主要有三个步骤：
在$k$时刻
- 估计或测量读取当前的系统状态
- 基于$u_k,u_{k+1},\cdots,u_{k+N}$来进行最优化，我们设定一个目标代价函数
  $$J = \sum\limits_k^{N - 1} (\boldsymbol{e}^T_k\boldsymbol{Qe}_k + \boldsymbol{u}^T_k\boldsymbol{Ru}_k + \boldsymbol{e}^T_N\boldsymbol{Fe}_N)
  $$其中第三项是**终端误差代价**，$\boldsymbol{F}$是**终端代价权重矩阵**，随后对整个目标代价函数进行最优化，同时在求解这个问题时还需要附上系统的约束
  数据获取过程具体来说就是设定一个**控制区间**，在这个区间内去预测例如$u_k,u_{k+1},u_{k+2}$的输入的结果$y_{k+1},y_{k+2},y_{k+3}$，这些输入的输出所在区间为**预测区间**
- 最后，我们只选取上一步预测的$u_k$作为$k$时刻的输入，而舍弃后面时刻，随后我们均将两个区间向右移动一时刻，继续上述步骤，这就叫**滚动优化控制**$(\mathbf{Receding~Horizon~Control})$