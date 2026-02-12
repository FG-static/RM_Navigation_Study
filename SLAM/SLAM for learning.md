## $$\mathbf{SLAM}~学习笔记$$

- 单目相机
  只使用一个摄像头进行SLAM的做法称为单目SLAM
  缺点是无法判断**尺度**$\mathbf{(SLAM)}-$估计值与真实值之间的因子，称为**尺度不确定性**
- 双目相机
  通过左右相机的**基线**-两个相机之间的距离，结合左右相机获得的图像计算真实像素点的深度（要计算所有像素点），所以双目SLAM的主要问题就是算力
- 深度相机
  通过发射和接收光线测定像素点的距离（深度），这种方法使用物理手段比较节省算力

## 视觉SLAM框架
经典视觉SLAM框架：
![alt text](Image//image.png)

### 视觉里程计
和里程计很像，在较短的时间内通过前几帧的图像信息估计相机运动并进行建图，缺点同理，会因为误差累积造成**漂移**，为了解决漂移，就需要**后端优化**和**回环检测**了

### 后端优化
后端优化就是如何从带有噪声的数据中估计整个系统的状态，以及这个状态估计的不确定性有多大——这称为最大后验概率估计（MAP），这里的状态既包括机器人也包括地图
所以SLAM问题的本质就是：**对运动主体本身和周围环境空间不确定性的估计**

#### 回环检测
用于解决**时间漂移问题**，通过判断图像之间的相似性，让机器人知道到达了（回到了）某个点，再对位置估计值进行修正，即可消除漂移

### $\mathrm{SLAM}$问题的数学化
我们定义：每一个运动的时刻为$t=1,\cdots,K$，每一个时刻的位置为$x=x_1,\cdots,x_K$，在运动过程中，每一个时刻机器人都会识别到若干个路标，不妨设总共有$N$个路标，分别是$y=y_1,\cdots,y_N$
接下来我们来定义**运动**和**观测**

#### 运动
也就是从$k-1$时刻到$k$时刻位置$x$的变化，于是我们可以设定一个**运动方程**
$$\bold{x}_k=f(\bold{x}_{k-1},\bold{u}_k,\bold{w}_k)$$
其中$\bold{u}_k$就是$k$时刻运动传感器的输入，而$\bold{w}_k$就是$k$时刻的噪声，$f$用于描述整个过程

#### 观测
描述机器人在$k$时刻$x_k$位置探测到了某一个路标$y_j$
和运动一样，我们可以写一个**观测方程**
$$\bold{z}_{k,j}=h(\bold{y}_j,\bold{x}_k,\bold{v}_{k,j})$$
其中$\bold{z}_{k,j}$是一个$k$时刻的观测数据，$\bold{v}_{k,j}$是观测噪声

#### 综合
综合运动方程和观测方程
$$\begin{cases}
  \bold{x}_k=f(\bold{x}_{k-1},\bold{u}_k,\bold{w}_k),~k=1,\cdots,K\\
  \bold{z}_{k,j}=h(\bold{y}_j,\bold{x}_k,\bold{v}_{k,j}),~(k,j)\in \mathcal{O}
\end{cases}$$
这就是SLAM过程的两个基本方程，其中$\mathcal{O}$是一个集合，记录了哪个时刻观测到了哪个路标
按照运动观测噪声的线性和非线性、噪声的高斯和非高斯分布来分类，可以将其分为**线性/非线性**和**高斯/非高斯**系统

## 三维空间刚体运动
### 平移和旋转
对于世界坐标系与相机坐标系，假设它们分别由两组正交基组成$[\bold{e}_1,\bold{e}_2,\bold{e}_3]$和$[\bold{e}'_1,\bold{e}'_2,\bold{e}'_3]$
对于一个在三维空间中的向量$\bold{p}$，它在不同坐标系下的坐标不同，它不会随着坐标系变换而变换，这两个坐标系之间相差了一个**欧式变换**

#### 旋转
$\bold{p}$在$[\bold{e}_1,\bold{e}_2,\bold{e}_3]$下的坐标表示为$\begin{bmatrix}
  p_1 \\
  p_2 \\
  p_3
\end{bmatrix}$，在$[\bold{e}'_1,\bold{e}'_2,\bold{e}'_3]$下的坐标表示为$\begin{bmatrix}
  p'_1 \\
  p'_2 \\
  p'_3
\end{bmatrix}$，由于$\bold{p}$本身并不会变，所以满足$$[\bold{e}_1,\bold{e}_2,\bold{e}_3]\begin{bmatrix}
  p_1 \\
  p_2 \\
  p_3
\end{bmatrix}=[\bold{e}'_1,\bold{e}'_2,\bold{e}'_3]\begin{bmatrix}
  p'_1 \\
  p'_2 \\
  p'_3
\end{bmatrix}$$
只要我们两边左乘$\begin{bmatrix}
  \bold{e}^T_1 \\
  \bold{e}^T_2 \\
  \bold{e}^T_3
\end{bmatrix}$左边的系数就变成了单位矩阵
$$\begin{bmatrix}
  p_1 \\
  p_2 \\
  p_3
\end{bmatrix}=\begin{bmatrix}
  \bold{e}^T_1\bold{e}'_1 & \bold{e}^T_1\bold{e}'_2 & \bold{e}^T_1\bold{e}'_3 \\
  \bold{e}^T_2\bold{e}'_1 & \bold{e}^T_2\bold{e}'_2 & \bold{e}^T_2\bold{e}'_3 \\
  \bold{e}^T_3\bold{e}'_1 & \bold{e}^T_3\bold{e}'_2 & \bold{e}^T_3\bold{e}'_3 
\end{bmatrix}\begin{bmatrix}
  p'_1 \\
  p'_2 \\
  p'_3 
\end{bmatrix}$$
我们设中间那个大的矩阵为$\bold{R}$，这个矩阵由两组基之间的内积组成，所以其仅依赖于两对正交基的选取，故也称为**旋转矩阵**，也叫它**方向余弦矩阵**，实际上旋转矩阵是一个行列式为$1$的正交矩阵，反之行列式为$1$的正交矩阵也是一个旋转矩阵，所以对于$n$维旋转矩阵的集合可以如此定义：
$$\mathrm{SO}(n)={\bold{R}\in\mathbb{R}^{n\times n}|\bold{RR}^T=\bold{I},\mathrm{det}(\bold{R})=1}$$
其中$\mathrm{SO}(n)$是特殊正交群，由于$\bold{R}$是一个正交矩阵，所以它的逆（也是转置）则描述了一个相反的旋转：
$$\bold{a}'=\bold{R}^{-1}\bold{a}=\bold{R}^T\bold{a}$$

#### 平移
除旋转外，对于平移我们只需要一个额外的平移向量即可，按上面的式子我们可以设出坐标系$1$和坐标系$2$，可以得出：
$$\bold{a}_1=\bold{R}_{12}\bold{a}_2+\bold{t}_{12}$$
，其中$\bold{t}$就是平移向量（坐标系原点指向另一个坐标系原点的向量），旋转矩阵的下标表示**把从坐标系2的向量变换到坐标系1中**，注意这里的下标是从右往左读，而平移向量是从左往右读的

#### 变换矩阵与齐次坐标
上述变换的描述正确但不是线性变换，这样做的后果是多次变换后式子会变得麻烦不堪
我们设**变换矩阵**$\bold{T}=\begin{bmatrix}
  \bold{R} & \bold{t} \\
  \bold{0}^T & 1
\end{bmatrix}$并重写$\bold{a}$为$\begin{bmatrix}
  \bold{a} \\
  1
\end{bmatrix}$，$\bold{a}'$同理，称它们为**齐次坐标**。这样我们就能重写之前的变换公式：
$$\begin{bmatrix}
  \bold{a}' \\
  1
\end{bmatrix}=\begin{bmatrix}
  \bold{R} & \bold{t} \\
  \bold{0}^T & 1
\end{bmatrix}\begin{bmatrix}
  \bold{a} \\
  1
\end{bmatrix}=\bold{T}\begin{bmatrix}
  \bold{a} \\
  1
\end{bmatrix}$$
这样这个变换就是一个线性变换了，对于多次变换也只需要多次左乘矩阵即可，**我们约定如果直接写$\bold{b}=\bold{T}\bold{a}$那么默认它们已经进行了齐次坐标化**（如果是$\bold{b}=\bold{Ra}$则不使用齐次坐标）
对于$\bold{T}$，这种矩阵属于**特殊欧式群**$\mathrm{SE}(n)$，在这里有：
$$\mathrm{SE}(3)=\begin{Bmatrix}
  \bold{T}=\begin{bmatrix}
    \bold{R} & \bold{t} \\
    \bold{0}^T & 1
  \end{bmatrix}\in\mathbb{R}^{4\times4}|\bold{R}\in\mathrm{SO}(3),\bold{t}\in\mathbb{R}^3
\end{Bmatrix}$$
与$\mathrm{SO}(3)$一样，该矩阵的逆表示一个反向变换：
$$\bold{T}^{-1}=\begin{bmatrix}
    \bold{R}^T & -\bold{R}^T\bold{t} \\
    \bold{0}^T & 1
  \end{bmatrix}$$
它的下标和旋转矩阵的下标表示方法一致
在这里先给一个符号定义，记$\bold{a}^\land=\begin{bmatrix}
  0 & -a_3 & a_2 \\
  a_3 & 0 & -a_1 \\
  -a_2 & a_1 & 0 
\end{bmatrix}$为向量$\bold{a}=\begin{bmatrix}
  a_1 \\
  a_2 \\
  a_3
\end{bmatrix}$的**反对称矩阵**
我们注意到，虽然$\bold{R}$能通过两组基来描述两个基坐标系之间的旋转变换，但是我们也可以设一个三维向量$\bold{n}$用$\theta\bold{n}$描述旋转轴为$\bold{n}$旋转角度为$\theta$的旋转变换（默认按右手定则），我们可以从**罗德里格斯公式**$(\mathrm{Rodrigues's~Formula})$：
$$\bold{R}=\cos\theta\bold{I}+(1-\cos\theta)\bold{nn}^T+\sin\theta\bold{n}^\land$$
得到旋转矩阵和旋转向量之间的转换公式，同时如果我们两边同时**求迹**，能得到角度的求解公式：
$$\theta=\arccos\cfrac{\mathrm{tr}(\bold{R})-1}{2}$$

#### 四元数
由于不存在不带奇异性的三维向量描述旋转的方式，我们可以使用四元数这种一个实部三个虚部的类复数来表示旋转：$$\bold{q}=q_0+q_1i+q_2j+q_3k$$
其中三个虚部满足以下表达式：
$$\begin{cases}
  i^2=j^2=k^2=-1 \\
  ij=k,ji=-k \\
  jk=i,kj=-i \\
  ki=j,ik=-j
\end{cases}$$
我们可以用一个标量一个向量表示四元数：
$$\bold{q}=[s,\bold{v}]^T,~s=q_0\in\mathbb{R},~\bold{v}=[q_1,q_2,q_3]^T\in\mathbb{R}^3$$
$s$称为四元数的实部，$\bold{v}$称为四元数的虚部，大部分性质和普通复数一样，四元数的逆被定义为：
$$\bold{q}^{-1}=\cfrac{\bold{q}^*}{||\bold{q}||^2}$$
其中$\bold{q}^*$为$\bold{q}$的共轭
对于三维点$p$用四元数表示它（将坐标分别设为四元数的三个虚部系数），用四元数$\bold{q}$描述从$p$变换到$p'$的旋转变换，满足以下式子：
$$p'=\bold{q}p\bold{q}^{-1}$$
在这里需要构造的四元数为$\bold{q}=\cos\cfrac{\theta}{2}+\bold{v}\sin\cfrac{\theta}{2}$
如果不构造这种形式的旋转公式，只乘一次四元数就会导致实部非0，这样操作可以让实部为0并且旋转角度叠加为$\theta$，这是由于$\bold{q}^{-1}=\cos\cfrac{\theta}{2}-\bold{v}\sin\cfrac{\theta}{2}=\cos(-\cfrac{\theta}{2})+\bold{v}\sin(-\cfrac{\theta}{2})$
而左乘正转右乘逆转，所以这里确实是$\cfrac{\theta}{2}-(-\cfrac{\theta}{2})=\theta$
同时四元数也能写成矩阵，我们定义符号$^+$和$^\oplus$：
$$\bold{q}^+=\begin{bmatrix}
  s & -\bold{v}^T \\
  \bold{v} & s\bold{I}+\bold{v}^\land
\end{bmatrix},~\bold{q}=\begin{bmatrix}
  s & -\bold{v}^T \\
  \bold{v} & s\bold{I}-\bold{v}^\land
\end{bmatrix}$$
所以四元数的乘法也可以由上述形式写成矩阵的乘法：
$$\bold{q}^+_1\bold{q}_2=\begin{bmatrix}
  s_1 & -\bold{v}^T_1 \\
  \bold{v}_1 & s_1\bold{I}+\bold{v}^\land_1
\end{bmatrix}\begin{bmatrix}
  s_2 \\
  \bold{v}_2
\end{bmatrix}=\begin{bmatrix}
  -\bold{v}^T_1\bold{v}_2+s_1s_2 \\
  s_1\bold{v}_2+s_2\bold{v}_1+\bold{v}^\land_1\bold{v}_2
\end{bmatrix}=\bold{q}_1\bold{q}_2$$
同理能得$\bold{q}_1\bold{q}_2=\bold{q}^\oplus_2\bold{q}_1$
我们回到四元数的旋转变换公式：
$$\bold{p}'=\bold{qpq}^{-1}=\bold{q}^+\bold{p}^+\bold{q}^{-1}=\bold{q}^+(\bold{q}^{-1})^\oplus\bold{p} \\
=\begin{bmatrix}
  1 & \bold{0} \\
  \bold{0}^T & \bold{vv}^T+s^2\bold{I}+2s\bold{v}^\land+(\bold{v}^\land)^2
\end{bmatrix}\bold{p}$$
注意最后一个矩阵的右下角正是旋转矩阵$\bold{R}=\bold{vv}^T+s^2\bold{I}+2s\bold{v}^\land+(\bold{v}^\land)^2$
我们两边同时求迹：
$$\mathrm{tr}(\bold{R})=4s^2-1$$
同时：
$$\theta=\arccos\cfrac{\mathrm{tf}(\bold{R}-1)}{2} 
  = \arccos(2s^2-1) =2\arccos s$$
于是我们得到四元数$\bold{q}=\begin{bmatrix}
  q_0 \\
  q_1 \\
  q_2 \\
  q_3
\end{bmatrix}$的旋转公式：
$$\begin{cases}
  \theta=2\arccos q_0 \\
  \begin{bmatrix}
    n_x \\
    n_y \\
    n_z
  \end{bmatrix}=\cfrac{1}{\sin\frac{\theta }{2}}\begin{bmatrix}
    q_1 \\
    q_2 \\
    q_3
  \end{bmatrix}
\end{cases}$$
其中$[n_x, n_y, n_z]^T$是单位旋转轴

#### 几何变换
![alt text](Image//image-1.png)

## 李群与李代数
### 正式定义
#### 群
前文讲述了三维旋转矩阵构成了特殊正交群$\mathrm{SO}(3)$，变换矩阵构成了特殊欧氏群$\mathrm{SE}(3)$
可以注意到，这两个群都不满足加法封闭性，但是满足乘法封闭性，我们称这种**只有一个良好运算的集合**为**群**，它是一种集合加上一种运算的**代数结构**，我们把集合记作$A$，运算记为$\cdot$，则群可以记作$G = (A, \cdot)$，它拥有四个性质
- 封闭性：$\forall a_1,a_2\in A,a_1\cdot a_2\in A$
- 结合律：$\forall a_1,a_2,a_3\in A,(a_1\cdot a_2)\cdot a_3=a_1\cdot(a_2\cdot a_3)$
- 么元：$\exists a_0 \in A, \forall a \in A, a_0 \cdot a = a \cdot a_0 = a$
- 逆：$\forall a \in A, \exist a^{-1} \in A, a \cdot a^{-1} = a_0$

**李群**是指具有连续（光滑）性质的群，$\mathrm{SO}(n)$和$\mathrm{SE}(n)$在实数空间上连续，它们都是李群，我们先从李代数入手再引出李群

#### 李代数
