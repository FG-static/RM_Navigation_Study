## $\mathbf{MPC}-模型预测控制$
### 基本概念
$r(t)-\mathrm{Reference}$到$u(t)-\mathrm{Input}$到$y(t)-\mathrm{Output}$再回到$r(t)$，是一个**单输入单输出系统**$\mathrm{(SISD)}$，定义误差$e=y-r$，从轨迹追踪的角度来说：
- $\int^t_0 e^2 dt$越小，误差越小，追踪越好
- $\int^y_0 u^2 dt$越小，输入越小，系统越容易以更少的能量稳定