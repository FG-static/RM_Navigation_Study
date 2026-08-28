# $\text{Loop-Closure Detection, LCD & Pose-Graph Optimization, PGO}$
目前大部分前端里程计主要依赖雷达惯导紧耦合方案（如 FAST-LIO 系列），但在类似长廊、大空旷地带或高动态比赛场景下，单靠前端容易发生几何退化或累计漂移。
针对这种问题，可以通过引**入后端优化**（如 Pose Graph Optimization, PGO）与**回环检测机制（LCD）**（如 Scan Context、Radius Search 或基于物理特征的方法）来约束累计误差，精化历史轨迹。
实际比赛场景中，Sentry 机器人需要在多障碍、长距离的赛道上长时间稳定导航，对建图与定位的全局一致性有着极高的要求。

## $\text{LCD}$
回环检测最常用的方法就是**Scan Context**，即在俯视视角下将点云变为描述子，进行匹配；
最简单的方法是**位姿距离回环**，即通过计算当前帧与历史帧之间的位姿距离来判断是否发生回环；
近期提出的更好的方法**LIDAR Iris**，它是Scan Context的改进版，通过引入更多点云的几何特征来提高回环检测的准确性

### $\text{Scan Context}$

### $\text{LiDAR Iris}$

> **论文**：LiDAR Iris for Loop-Closure Detection (Wang et al., IROS 2020)
> **核心思想**：借鉴人虹膜识别（Iris Recognition）的思路，把每一帧3D LiDAR点云"拍扁"成一张2D极坐标图像，再用生物特征识别里的经典方法（Gabor滤波 + 二值化）提取出一个极紧凑的二值描述子，最后用Hamming距离做超快速匹配。

#### 为什么需要LiDAR Iris？—— 与Scan Context的对比

Scan Context的三个主要缺陷：
1. **信息丢失严重**：它只记录每个bin中点云的**最大高度值**，相当于把一个bin里所有点的高度信息"压缩"成了一个数，丢失了点云的内部结构
2. **不具有旋转不变性**：两帧点云如果朝向不同，Scan Context需要**暴力旋转匹配**（逐角度对比），计算量大
3. **缺少特征提取**：原始Scan Context直接用连续值描述子匹配，没有经过特征增强处理

LiDAR Iris针对性地解决了这三个问题。用一个生活中的类比：
- Scan Context像是**粗略的剪影**——只看轮廓，丢失细节
- LiDAR Iris像是**精细的指纹**——保留纹理细节，还做了二值化处理使匹配更快

#### 整体流程概览
```
3D LiDAR点云
    ↓ ①鸟瞰投影（从上往下看）
2D鸟瞰平面点云
    ↓ ②极坐标离散化（像雷达扫描一样划分bin）
LiDAR-Iris图像（一张2D灰度图）
    ↓ ③Fourier变换（消除朝向差异）
频域表示
    ↓ ④LoG-Gabor滤波（提取纹理特征）
滤波响应
    ↓ ⑤二值化（转为0和1）
二值签名图像（最终描述子）
    ↓ ⑥Hamming距离匹配
回环检测结果
```

#### 第一步：LiDAR-Iris图像生成

**1. 鸟瞰投影**

想象你站在一栋高楼的楼顶，垂直往下看，看到的就是鸟瞰视角。同样地，我们把3D点云从上往下"压"到2D平面上。

具体操作：取一个以LiDAR为中心、边长为 $k$ 米的正方形区域（论文中 $k = 80\text{m}$），只保留这个区域内的点云，其余全部丢弃。为什么选80m？因为这是LiDAR的有效感知范围，太远的点噪声太大、太近的点信息量少。

**2. 极坐标离散化**

这是LiDAR Iris最关键的一步。我们不是简单地把正方形区域划成网格，而是用**极坐标**来划分——就像雷达扫描一样，以LiDAR为圆心，向外辐射。

具体来说：
- **径向（radius）**：从圆心向外，每隔1m画一个同心圆，共80个径向bin
- **角向（angular）**：绕圆心一圈360°，每隔1°画一条射线，共360个角向bin

这样整个区域就被分成了 $80 \times 360 = 28800$ 个小格子（bin）。

**3. 高度编码——8位二进制码**

每个bin里可能有多个3D点，这些点有不同的高度值。怎么把高度信息编码成一个像素值？

LiDAR Iris的做法是：
1. 确定高度范围 $[y_l, y_h]$（对于HDL-64E：$y_l = -3\text{m}, y_h = 5\text{m}$，因为LiDAR安装在车顶约3m高处）
2. 将这个范围**线性划分为8个子bin**
3. 对于每个子bin，如果其中有至少一个点，就标记为1，否则为0
4. 8个子bin得到8位二进制码，转换为0~255的十进制数

**举个例子**：
假设某个bin内有3个点，高度分别是-1m、2m、4m：
- 子bin范围：[-3,-2), [-2,-1), [-1,0), [0,1), [1,2), [2,3), [3,4), [4,5)
- 对应二进制：0, 0, 1, 0, 0, 1, 0, 1
- 十进制值：$0 \times 128 + 0 \times 64 + 1 \times 32 + 0 \times 16 + 0 \times 8 + 1 \times 4 + 0 \times 2 + 1 \times 1 = 37$

而Scan Context在这种情况下只会记录最大高度4m，完全丢失了-1m和2m这两个点的信息。

最终，所有bin的像素值组成一张 $80 \times 360$ 的灰度图，这就是**LiDAR-Iris图像**。之所以叫"Iris"（虹膜），是因为从上往下看LiDAR点云的形状，和人眼睛的虹膜非常相似——都是同心圆结构。

这里借鉴了Daugman虹膜识别中的**Rubber Sheet Model**思想：把圆形的虹膜区域展开为一条矩形图像条（将极坐标 $(r, \theta)$ 映射为矩形图像的行列坐标）。LiDAR Iris同样将鸟瞰视角的点云"展开"为一条 $80 \times 360$ 的图像条，每个像素的强度就是该bin的8位二进制码对应的十进制值。

这种编码方式相比传统的基于直方图的全局描述子（如ESF、M2DP等）有两个显著优势：
1. **不需要逐点计数**：直方图类方法需要统计每个bin内有多少个点，而LiDAR-Iris只关心每个子bin"有没有点"（0或1），计算更高效
2. **无需预训练**：编码规则是固定的手工设计，不需要像PointNetVLAD等深度学习方法那样依赖大量训练数据，泛化性更好

需要注意的是，到这一步得到的描述子**尚未具有旋转/yaw不变性**——同一位置不同朝向采集的两帧，生成的LiDAR-Iris图像会有水平方向的角向循环偏移。下面将通过Fourier变换和循环匹配降低这个影响。真实世界的横向/纵向平移不能简单等价为这种循环偏移。

#### 第二步：Fourier变换、循环位移与旋转不变性

**问题是什么？**

机器人在实际运行中，两次经过同一地点时，朝向不可能完全一致。朝向差异会影响LiDAR-Iris图像的匹配。

具体来说，在LiDAR-Iris图像中：
- **旋转（heading变化）**：机器人绕自身z轴旋转一个角度 $\Delta\theta$，相当于所有点的方位角都偏移了 $\Delta\theta$。由于LiDAR-Iris图像的水平轴就是方位角（0°~360°离散为360个bin），所以旋转直接表现为图像在**水平方向上的循环平移**。比如机器人右转了30°，整张图像就往右滚动了30个bin。

这个运动在LiDAR-Iris图像上体现为**角向循环平移（cyclic shift）**。我们需要一种方法，让匹配不受这种滚动的影响。

需要特别注意：**真实的$x/y$平移不等价于LiDAR-Iris图像上的简单循环平移**。LiDAR-Iris图像是以当前LiDAR为中心构造的极坐标描述子，机器人横向或纵向移动后，周围几何在极坐标下会发生非线性重投影和局部形变，而不是像yaw变化那样形成一个干净的水平循环shift。因此，LiDAR-Iris/Scan Context类方法更擅长估计和补偿yaw偏差，不应该把它理解为能可靠估计平面位移偏差。

**为什么Fourier变换能解决角向循环偏移？**

这是信号处理中的经典结论：**图像坐标中的平移，在频域中主要表现为相位变化，而幅度谱不变**。

具体来说，设两幅LiDAR-Iris图像为 $I_1$ 和 $I_2$，如果它们之间存在图像坐标意义上的平移 $(\delta_x, \delta_y)$，即 $I_1(x, y) = I_2(x - \delta_x, y - \delta_y)$。

对两者分别做2D Fourier变换，得到 $\hat{I}_1$ 和 $\hat{I}_2$，它们满足：
$$
\hat{I}_1(w_x, w_y) \cdot e^{-i(w_x \delta_x + w_y \delta_y)} = \hat{I}_2(w_x, w_y)
$$

这个公式的含义是：$I_2$ 的频谱等于 $I_1$ 的频谱乘以一个相位因子 $e^{-i(w_x \delta_x + w_y \delta_y)}$。如果我们计算**归一化互功率谱**：
$$
Corr = \frac{\hat{I}_2 \cdot \hat{I}_1^*}{|\hat{I}_1| \cdot |\hat{I}_2|} = e^{-i(w_x \delta_x + w_y \delta_y)}
$$

其中 $*$ 表示复共轭。再对这个结果做**逆Fourier变换**：
$$
Corr(x, y) = F^{-1}(Corr) = \delta(x - \delta_x, y - \delta_y)
$$

结果是一个脉冲函数，只在 $(\delta_x, \delta_y)$ 处非零。这意味着：
- 对于LiDAR-Iris这种角向展开图，yaw变化对应的水平循环偏移可以通过类似相位相关/循环匹配的思想估计出来；
- Fourier变换的幅度谱对图像坐标上的循环平移具有不变性，所以可以降低朝向差异对描述子匹配的影响。

**通俗理解**：就像你把一首歌在播放器里往后拖了几秒再听，虽然"波形"变了，但"频谱"（频率成分）是不变的。LiDAR Iris利用的就是这个原理来减弱角向循环偏移的影响。它并不保证能区分或估计真实世界中的横向/纵向平移，尤其在长廊、隧道、重复墙面这类结构里，两个物理位置不同但局部几何相似的扫描仍可能得到很近的描述子距离。

#### 第三步：LoG-Gabor滤波提取二值特征

**为什么需要滤波？**

Fourier变换后的幅度谱虽然对图像坐标里的循环位移更鲁棒，但直接用它做匹配效果不够好——LiDAR-Iris图像本身存在点密度变化、传感器噪声、小范围视角变化导致的强度波动等问题，直接比较像素值不够稳定。我们需要一层特征提取，把关注点从"这个像素值是多少"转移到"这个区域有没有某种结构变化"。

**前置知识：Gabor滤波器**

Gabor滤波器可以理解为**高斯窗口 × 正弦波**：
$$
g(x, y) = \text{Gaussian}(x, y) \times \cos/\sin(\text{某个方向、某个频率})
$$

其中高斯窗口就是二维高斯函数 $\text{Gaussian}(x, y) = e^{-(x^2 + y^2)/(2\sigma^2)}$，它的作用是给正弦波加一个"衰减罩"——中心处权重最大，越往边缘权重越小，最终平滑地衰减到零。这样做的目的是让滤波器**只关注局部区域**：正弦波负责提取特定频率和方向的纹理，高斯窗口负责把这个提取能力限制在局部范围内，避免远处无关的纹理干扰。没有高斯窗口的话，正弦波在整个图像上都有响应，就失去了"局部性"。

它对图像中**特定方向、特定频率**的纹理/边缘结构敏感。比如一条竖直边缘会让某个方向的Gabor滤波器产生强响应，而横向纹理则让另一个方向的滤波器响应强。因此Gabor滤波器广泛用于虹膜识别、指纹识别、纹理分析等场景——LiDAR Iris名字里的"Iris"正是借鉴了虹膜识别的思路。

**注意命名混淆**：LoG-Gabor ≠ LoG（Laplacian of Gaussian）。LoG是拉普拉斯高斯，用于边缘检测；本文的LoG-Gabor是**Log-Gabor**，即对数域的Gabor滤波器，两者完全不同。

**Log-Gabor vs 普通Gabor**

普通Gabor滤波器在**线性频率轴**上定义高斯形状，而Log-Gabor改为在**对数频率轴**上定义：
$$
G(f) = \exp\left(\frac{-(\log(f/f_0))^2}{2(\log(\sigma/f_0))^2}\right)
$$
其中 $f_0$ 是中心频率，$\sigma$ 控制带宽，$\sigma/f_0$ 需保持恒定。

这样做的核心好处是**没有直流分量（DC component）**——普通Gabor在零频附近有响应，会对图像整体亮度/低频偏置敏感；Log-Gabor把这个"盲区"去掉了，对强度偏移更鲁棒。对于LiDAR-Iris这种点密度和强度会随帧变化的图像来说，这一点很关键。

**为什么Log-Gabor适合LiDAR-Iris？**

LiDAR-Iris图像直接比较有几个问题：
- 点云密度会变化（同一位置不同时间，点数不同）
- 传感器噪声导致局部像素值波动
- 小范围位移造成强度轻微变化

Log-Gabor滤波器关心的不是像素的绝对值，而是**局部有没有某种尺度上的结构响应**——比如这里有没有明显边界？有没有周期性结构？有没有某个尺度上的突变？这比直接比较原始像素值稳定得多。

**具体操作**：
1. 使用8个不同尺度的1D Log-Gabor滤波器，对LiDAR-Iris图像的**每一行**进行卷积（实际做法：图像 → FFT → 频域乘以Log-Gabor滤波器 → IFFT → 得到滤波响应）。每个滤波器的波长按相同因子递增（就像用从细到粗的8把"刷子"分别刷一遍）
2. 取卷积响应的**实部和虚部**作为特征
3. 实验发现，使用**前4个**滤波器就能达到最佳性能（更多滤波器只会增加计算量，不提升精度）

**二值化**：

将4个滤波器的卷积响应，通过**简单的阈值操作**（以均值为阈值）转换为0和1：
- 响应值 > 均值 → 1
- 响应值 ≤ 均值 → 0

这和人虹膜识别中经典的**IrisCode**完全一样——IrisCode就是对虹膜图像用Gabor滤波后二值化得到的。LiDAR Iris把同样的思路搬到了LiDAR点云上。

二值化的好处：
1. **极快的匹配**：XOR + popcount算Hamming距离，比浮点距离快几个数量级
2. **对强度变化鲁棒**：只看响应的正负/相位，不看幅值大小，小的强度波动不一定改变bit值
3. **紧凑**：每个位置的多通道响应组合成8-bit code，保留的是局部结构模式而非原始像素强度

最终，每个LiDAR-Iris图像得到一个紧凑的**二值签名图像**（binary signature image），这就是LiDAR Iris的最终描述子。

> **注意**：Log-Gabor滤波和前面的Fourier变换都用到了频域工具，但用途不同——Fourier/相位相关主要用于处理描述子图像上的循环位移，工程里最重要的是估计yaw相关的角向shift；Log-Gabor用于**提取纹理结构响应**（频域滤波）。不要把这个"图像循环位移"误解成真实世界$x/y$平移的可靠估计。

#### 第四步：Hamming距离匹配

二值描述子的匹配极其高效，只需要三步：
1. **XOR（异或）**：两个二值描述子逐位异或，不同的位为1，相同的位为0
2. **popcount（位计数）**：统计异或结果中1的个数，这就是Hamming距离
3. **阈值判断**：如果Hamming距离 < 阈值 $d_f$，则判定为回环

为什么这么快？因为XOR和popcount都是CPU的**单指令操作**，现代处理器上执行一次只需要几个时钟周期。相比之下，Scan-Context需要计算欧氏距离或余弦相似度，涉及浮点运算，慢得多。

**举个例子**：
- 描述子A：`10110010 01101001`
- 描述子B：`10100011 01101000`
- XOR结果：`00010001 00000001`
- Hamming距离：3（有3个不同的位）

#### 回环检测完整流程

1. **建库**：机器人运行过程中，每个关键帧都提取一个LiDAR-Iris二值描述子，存入数据库
2. **查询**：对当前帧提取描述子，与数据库中所有历史帧（排除最近30帧，避免 trivial self-match）计算Hamming距离
3. **筛选**：Hamming距离小于阈值的帧作为回环候选
4. **验证**：通过几何验证（ground truth距离 < 4m）确认真阳性

论文定义了两个评估协议：
- **Protocol A**（实时检测）：判断当前帧是否回到了之前访问过的位置
- **Protocol B**（地点重识别）：构建所有帧对的affinity矩阵，评估整体匹配能力

#### 实验结果

**数据集**：
- KITTI odometry：序列00, 05, 08（HDL-64E，64通道LiDAR）
- 自采集VLP-16数据集：校园环境（VLP-16，16通道LiDAR）

**对比方法**：Scan-Context、M2DP、ESF

**关键结论**：
- 在所有5个序列上，LiDAR Iris的precision-recall曲线均优于或持平于Scan-Context，**显著优于**M2DP和ESF
- ESF表现最差，因为它强烈依赖可见区域的结构差异，在结构相似的环境中容易混淆
- M2DP在**反向回环**（opposite direction，即从相反方向经过同一地点）场景下表现不佳

**计算效率**：
- LiDAR Iris：平均 **0.0231s**/帧（特征提取+匹配）
- Scan-Context：**0.0257s**/帧
- LiDAR Iris不仅更快，而且不需要设置候选参数（如Scan-Context的ring-key tree中的候选数量）

#### 优缺点总结

**优点**：
| 特性 | 说明 |
|------|------|
| 二值描述子 | 极紧凑，Hamming距离匹配极快，适合实时SLAM |
| 角向循环偏移鲁棒 | Fourier/循环匹配降低yaw变化造成的水平循环偏移影响 |
| 光照无关 | LiDAR本身不受光照和季节变化影响 |
| 无需训练 | 手工设计特征，不需要大量训练数据，泛化性好 |
| 高度编码保留更多信息 | 8位二进制码 vs Scan-Context的单一最大高度值 |
| 跨传感器适用 | 适用于HDL-64E和VLP-16等不同LiDAR |

**缺点**：
| 特性 | 说明 |
|------|------|
| 主要适合平面yaw回环 | 描述子匹配主要处理平面场景和朝向偏移，完整6DoF一致性需要IMU、GICP或其他几何验证补充 |
| 垂直FOV敏感 | 编码参数（$y_l, y_h$）需要根据具体LiDAR传感器调整 |
| 全局描述子 | 无法提供局部特征匹配能力 |
| 高度离散化粗糙 | 8个子bin的高度分辨率有限 |
| 对真实平移不可靠 | 横向/纵向位移不是简单循环shift，长廊等重复结构中容易产生假回环候选 |

#### 长廊假回环的本质

在长廊中，两次经过的机器人轨迹可能并不是同一条物理轨迹，而是存在一定横向偏移。但由于两侧墙面、地面、天花板等局部结构高度相似，LiDAR-Iris可能仍然给出很低的描述子距离。此时：

1. Iris只说明"局部几何描述子很像"，不能证明它们是同一个物理位置；
2. Iris返回的`yaw_bias`主要是角向循环偏移，不是完整的$x/y$位移估计；
3. GICP会主动寻找一个刚体变换让两片点云尽量贴合，因此可能把真实存在的横向轨迹差当成配准误差修掉；
4. 如果这条假回环边进入PGO，优化器会认真满足这条错误约束，把两条本来不同的长廊轨迹拉到一起，导致地图错位。

因此，LCD不能只依赖"Iris距离 + GICP score"。实际系统中还需要：
- submap-to-submap几何验证，而不是单帧对单帧；
- 对GICP修正量做分轴gate，尤其限制不合理的横向/lateral correction；
- sequence consistency，即连续几帧都支持同一段历史轨迹时才加入回环边；
- top-K候选，而不是只相信Iris距离最小的单个候选。

### 工程化的 LCD 设计

#### 分级检索：先粗召回，再精匹配

当历史帧数量较少时，可以把当前描述子与所有历史描述子逐一比较。但随着运行时间增加，全量精匹配会越来越慢，而且大量相似帧会让后续几何验证负担过重。更常见的工程做法是建立两级检索：

    当前点云
        ↓
    轻量的粗检索键
        ↓
    历史数据库中选出少量 top-K 候选
        ↓
    完整描述子精匹配
        ↓
    点云几何验证
        ↓
    回环约束

“检索键”是从完整描述子压缩得到的低维特征，例如沿某个方向做平均、投影或降采样。它只负责快速找到大致相似的历史区域，不负责证明两帧一定是同一地点。真正的回环判断仍然需要完整描述子和几何配准。

在小规模数据库中，全量遍历检索键也可以作为第一版实现；数据库很大时，再把检索键放入 KD-tree 或其他近似近邻索引。无论采用哪种搜索方式，都要在插入新关键帧时同步更新索引，并保留时间间隔和行驶距离过滤，避免把相邻帧当成回环。

#### 反向检索与朝向相反的回环

如果检索键保留了方向上的排列关系，那么机器人从相反方向经过同一地点时，键中的元素顺序可能发生反转。此时不能只计算：

$$d(k_c,k_h)$$

还应计算当前检索键反转后的距离：

$$d(k_c^{reverse},k_h)$$

最终取两者较小值作为粗召回距离。这里的 $k_c$ 表示当前帧检索键，$k_h$ 表示历史帧检索键。反向检索只能解决描述子排列方向问题，不能解决真实的平移误差；后续仍需让旋转描述子估计航向偏置，并用几何配准验证位置。

#### 高频扫描雷达的描述子输入

对于需要经过一段时间才能形成较完整视场的扫描式或固态 LiDAR，单帧点云可能只是局部观测。此时可以将连续的少量关键帧拼成局部子地图，再从子地图生成描述子。窗口过短，点云仍然稀疏；窗口过长，则会把运动、位姿误差和动态物体累积进描述子。

因此，描述子子地图的长度应由时间跨度、行驶距离或关键帧数量共同限制，并配合体素降采样。描述子不必对每一帧原始雷达消息计算，而应在经过位移、旋转、时间间隔和点数筛选的稳定关键帧上计算。

#### 关键帧筛选、候选过滤与回环边稀疏化

这三个步骤作用不同：

1. **关键帧筛选**：决定哪些观测进入描述子数据库和图中，通常依据位移、旋转、最小时间间隔以及点数。
2. **候选过滤**：在描述子匹配前排除时间上过近或里程计距离过小的历史帧，避免自匹配。
3. **回环边稀疏化**：候选已经通过描述子和几何验证后，仍不必把同一片区域的每个候选都加入图中。可以要求相邻回环边之间具有最小关键帧间隔或最小行驶距离，并结合连续多帧一致性、历史区域去重等策略。

候选过滤解决“哪些帧值得比较”，回环边稀疏化解决“哪些已验证约束值得长期保留”。后者可以减少重复约束对图优化的过度拉扯，也能降低每次新增回环后的优化成本。

#### 历史帧复用限制与有向序列进度 Gate

重复坡道、长廊或分岔口可能让同一个历史子地图持续吸附多个当前关键帧。描述子和 GICP 对这些局部几何都可能给出较好分数，单纯依靠回环边稀疏化只能减少入图频率，不能阻止同一个错误 history 在间隔满足后再次进入 PGO。

第一项改动为历史帧复用限制。系统用哈希表记录每个 history 被选为最终回环候选的次数，并在达到 `loop_history_max_current_matches` 后停止让该 history 参与 LCD 检索。这里只停用检索，不从 `keyframes_` 删除实体，因为关键帧仍要用于 PGO 节点查询、双向子地图构造、优化结果发布和地图重建。

计数发生在 GICP 完成并选出最终候选之后、回环边稀疏化之前。即使候选随后因 current 间隔不足而没有加入 PGO，也会消耗 history 的复用额度。否则连续候选会一直被稀疏化跳过而不计数，等间隔达到阈值后仍可能把同一个假回环加入图中。

```text
描述子候选
    ↓
GICP 验证并选出最终候选
    ↓
history selected_count += 1
    ↓
达到上限后停止该 history 的后续 LCD 检索
    ↓
回环边稀疏化
    ↓
PGO
```

Garage bag 实测中，默认上限设为 2 后，同一个 history 与大量 current 重复连接的现象明显减少，部分原本被错误 history 抢占的 current 能够改选更合理的历史帧。该限制已经验证有效，但它属于次数保险丝：第一次和第二次假匹配仍会通过原有 gate，不能单独判断首次匹配真假。

第二项改动为有向序列进度 gate。它以上一条真正加入 PGO 的回环边

$$
(C_a,H_a)
$$

作为锚点。对后续候选 $(C_i,H_i)$，使用累计里程计轨迹长度计算：

$$
\Delta s_c=s(C_i)-s(C_a),
$$

$$
\Delta s_h=d\,[s(H_i)-s(H_a)],
$$

其中 $d\in\{+1,-1\}$ 表示 history 与 current 的行驶方向。同向经过历史轨迹时 $d=+1$，反向经过时 $d=-1$；代码根据锚点处 current 与 history 车体前向轴的点积确定方向。最终计算 history 的单边滞后量：

$$
e_{\text{lag}}=\Delta s_c-\Delta s_h.
$$

当

$$
e_{\text{lag}}>\tau_{\text{lag}}
$$

时拒绝候选。这里不能直接使用绝对值，因为当前要抑制的是“current 已向前行驶较远，而 history 仍停留在原位置或前进过慢”。history 进度超前时 $e_{\text{lag}}<0$，初版不会因此拒绝；若实测出现 history 向前跳跃型假回环，应另设独立的 lead gate，而不是混入同一个绝对值阈值。

该 gate 放在描述子候选产生之后、GICP 之前：

```text
Iris / Cart 候选
    ↓
有向序列进度 gate
    ↓ 仅通过者
多初值 GICP
    ↓
最终候选与 history 复用计数
    ↓
回环边稀疏化与 PGO
```

锚点只在回环边真正加入 PGO 后更新，稀疏化跳过的候选不会改变锚点，避免未经入图的中间结果持续改变参考。该方法仍依赖第一条回环边基本可信；如果锚点本身就是假回环，序列 gate 只能检查后续运动是否与这条错误序列一致，不能恢复绝对地点身份。

当前验证状态如下：

- `loop_history_max_current_matches`：已通过 Garage bag 验证，能够减少单个 history 的一对多连接；
- `loop_sequence_max_history_lag_m`：代码与日志已实现并通过编译，尚需使用同一 bag 复现段验证阈值；
- Iris、Cart、GICP 的单帧质量 gate 仍然保留，序列 gate 只补充时间连续性约束，不替代几何验证。

#### 在SLAM系统中的位置
```
前端里程计(LIO/LVO) → 后端优化(PGO) → [回环检测: LiDAR Iris] → 回环约束 → 后端优化修正
```

## $\text{PGO}$

### 基本概念

位姿图优化（Pose Graph Optimization, PGO）是SLAM后端的核心，通过最小化所有约束边的误差来修正轨迹漂移。

**位姿图结构**：
```
节点 (Vertex) = 关键帧的位姿 T = [R t; 0 1]
边 (Edge) = 两个位姿之间的约束（里程计边/回环边）

里程计边 (蓝色): kf0 → kf1 → kf2 → kf3 → kf4
回环边 (橙色): kf4 → kf0 (检测到回环)
```

**优化目标**：
$$
\min_{\mathbf{T}} \sum_{(i,j) \in \mathcal{E}} \mathbf{e}_{ij}^T \Omega_{ij} \mathbf{e}_{ij}
$$

其中：
- $\mathbf{e}_{ij}$：边 $(i,j)$ 的误差（观测相对位姿与估计相对位姿的差异）
- $\Omega_{ij} = \Sigma_{ij}^{-1}$：信息矩阵（协方差矩阵的逆，表示置信度）

### 信息矩阵与协方差矩阵

**协方差矩阵 $\Sigma$**：表示测量的不确定性，对角线元素是各维度的方差 $\sigma_i^2$

**信息矩阵 $\Omega = \Sigma^{-1}$**：表示测量的置信度，对角线元素是各维度的精度 $1/\sigma_i^2$

为什么用 $\Sigma^{-1}$ 而不是 $\Sigma$？
- 从高斯分布推导：$p(\mathbf{x}) \propto \exp(-\frac{1}{2}(\mathbf{x}-\boldsymbol{\mu})^T \Sigma^{-1} (\mathbf{x}-\boldsymbol{\mu}))$
- $\Sigma^{-1}$ 自然出现在指数项的分母上，代表归一化因子
- 优化目标是最小化加权误差：$\min \mathbf{e}^T \Omega \mathbf{e}$
- $\Omega$ 大 → 置信度高 → 权重大 → 优化器更"信任"这条边

**实际设置**：

早期实现可以用统一标量权重：
```cpp
Eigen::Matrix6d odom_info = Eigen::Matrix6d::Identity() * 100.0;
Eigen::Matrix6d loop_info = Eigen::Matrix6d::Identity() * 500.0;
```

但统一标量无法表达不同自由度的可靠性。当前实现更推荐使用6DoF对角信息矩阵：
```yaml
# order: [x, y, z, roll, pitch, yaw]
pgo_odom_info_diag: [150.0, 150.0, 50.0, 40.0, 40.0, 200.0]
pgo_loop_info_diag: [650.0, 650.0, 250.0, 150.0, 150.0, 850.0]
```

数值越大，表示优化器越信任该自由度上的约束。长廊等退化场景中，通常不应该让z/roll/pitch和x/y/yaw拥有同等权重。

### 回环约束的可靠性处理

#### 根据几何质量动态调整信息矩阵

固定的回环权重默认所有回环测量都一样可靠，但实际的点云重叠程度、视角差异和局部结构会变化。可以先建立一个基础回环信息矩阵，再根据几何配准质量分数缩放它：

$$
\alpha =
\operatorname{clip}\left(
\frac{s_{ref}}{\max(s,s_{floor})},
\alpha_{min},
\alpha_{max}
\right)
$$

$$
\Omega_{loop}=\alpha\Omega_{loop}^{base}
$$

其中 $s$ 是配准质量分数，通常分数越小表示误差越小；$s_{ref}$ 是认为“正常可靠”的参考分数，$s_{floor}$ 防止分母过小，$\alpha_{min}$ 和 $\alpha_{max}$ 限制缩放范围。这样低质量回环会自动减弱，高质量回环可以使用基础权重。

这个方法不是严格的概率协方差估计，因为很多配准分数没有经过统计标定。实际使用时必须在代表性数据上观察分数分布，并设置上下限，不能让一次异常低分把回环权重无限放大。

#### 只对回环边使用鲁棒核

回环检测仍可能产生错误匹配，或者正确回环的测量与当前轨迹存在较大残差。可以只给回环边附加 Huber 等鲁棒核：

$$
\rho(r)=
\begin{cases}
\frac{1}{2}r^2, & |r|\leq\delta \\
\delta\left(|r|-\frac{1}{2}\delta\right), & |r|>\delta
\end{cases}
$$

小残差时它与普通最小二乘相同，大残差时将损失从二次增长改为近似线性增长，从而降低异常回环对整张图的破坏力。里程计边通常应保持普通二次约束，因为它们构成连续轨迹；鲁棒核可以减弱错误回环，但不能把错误测量变成正确测量，所以仍需要描述子、几何质量和修正幅度门限。

#### 重力对齐与坡道场景

在坡道、地面或大面积墙面等场景中，点云对高度、横滚和俯仰的约束可能退化。建图开始时可以利用静止阶段的 IMU 平均加速度估计重力方向，将世界坐标系的竖直方向与重力对齐；如果 IMU 数据不可用，也可以使用经过验证的人工安装角。

回环测量不一定要强行使用完整六自由度。更稳妥的做法是让回环主要约束水平位置和绕重力轴的航向，把高度、横滚和俯仰交给 IMU/里程计约束。只有点云在六个自由度上都具有充分几何约束时，才使用完整的六自由度回环测量。重力对齐必须在描述子、回环测量、轨迹发布和地图重建之间保持一致，否则会出现轨迹与地图使用不同坐标基准的问题。

### 前后端的时间一致性

前端的 scan-to-map 可能利用历史局部地图修正当前状态，但如果修正只写入当前状态，而此前已经保存的关键帧仍保持旧位姿，就会出现：

    旧关键帧轨迹
        + 前端突然修正当前位姿
        = 新旧关键帧之间产生不连续

此时后端看到的不是一条一致的里程计链，而是带有内部跳变的关键帧序列。局部子地图、里程计边和回环边可能互相矛盾，单纯调小回环权重并不能从根本上解决问题。

常见处理方式有三种：

1. 在前端维护一个活动窗口，scan-to-map 的修正向窗口内关键帧和相关状态回溯传播；窗口边界之外的关键帧才冻结。
2. 将前端局部坐标系与后端全局坐标系分开，通过一个随时间更新的局部到全局变换传递修正，避免直接改写已经冻结的历史数据。
3. 让后端接收带时间关系的状态或子地图，并在后端统一处理历史修正，而不是只接收已经发生跳变的绝对关键帧位姿。

这是前后端接口设计问题，不是单独的 GICP 或 PGO 参数问题。无论选择哪种方案，都要保证用于建立里程计边、拼接子地图和重建地图的位姿具有同一时间语义。

### PGO 后的地图重建

图优化只改变关键帧位姿，不会自动移动已经存储在磁盘或内存中的点云。因此地图重建时应明确区分：

- 已经拥有优化位姿的关键帧：使用优化后的位姿变换点云；
- 尚未进入当前优化结果的尾部关键帧：可以暂时跳过，或使用最近一次全局修正作近似；
- 没有可靠优化结果的帧：不要无条件把原始位姿和优化位姿混合使用。

最终地图应由“点云数据 + 与其对应的同一版本位姿”重新变换、拼接和降采样。每次 PGO 后重建地图并记录优化帧数量、尾部处理方式和全局修正量，才能判断地图变好还是只是视觉上发生了整体移动。

### g2o 求解器架构

g2o 的优化器由三层组成：
```
┌─────────────────────────────────────┐
│         SparseOptimizer             │  ← 顶层：管理图结构
├─────────────────────────────────────┤
│      OptimizationAlgorithm          │  ← 中层：优化算法 (LM/GN)
├─────────────────────────────────────┤
│         BlockSolver                  │  ← 底层：分块稀疏矩阵
├─────────────────────────────────────┤
│        LinearSolver                  │  ← 最底层：Ax = b
└─────────────────────────────────────┘
```

- **LinearSolver**：求解线性方程组 $H\Delta x = b$（Eigen/CSparse/Cholmod）
- **BlockSolver**：把稀疏的 H 矩阵按节点分块，加速求解
- **OptimizationAlgorithm**：Levenberg-Marquardt（推荐，鲁棒）或 Gauss-Newton（快但可能发散）

### g2o 核心类型

**Vertex（节点）**：
```cpp
auto *v = new g2o::VertexSE3();  // 6DOF 位姿节点
v->setId(0);                     // 唯一标识
v->setEstimate(pose);            // 当前位姿估计（Isometry3d）
v->setFixed(true);               // 是否固定（第一个节点通常6DoF全固定）
```

**Edge（边）**：
```cpp
auto *e = new g2o::EdgeSE3();    // SE3 约束边
e->setVertex(0, v_from);         // 起点
e->setVertex(1, v_to);           // 终点
e->setMeasurement(relative_pose); // 观测的相对位姿
e->setInformation(info_matrix);   // 信息矩阵（置信度）
```

**Isometry3d**：Eigen 的刚体变换类型，保证旋转矩阵正交（$R^T R = I$）
```cpp
Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
T.linear() = R;           // 旋转部分
T.translation() = t;      // 平移部分
```

### 完整 PGO 流程

```
关键帧到达
    ↓
添加节点 (VertexSE3)
    ↓
添加里程计边 (EdgeSE3)
    ↓
检测回环 (LiDAR Iris)
    ↓
GICP 验证回环
    ↓
添加回环边 (EdgeSE3)
    ↓
执行优化 (g2o LM)
    ↓
读取优化结果
    ↓
更新位姿 / 修正地图
```

### PoseGraphOptimizationSummary

优化结果摘要：
```cpp
struct PoseGraphOptimizationSummary {
    bool success = false;        // 优化是否成功
    int iterations = 0;          // 迭代次数
    int node_count = 0;          // 节点数量
    int edge_count = 0;          // 边数量
    double initial_chi2 = 0.0;   // 优化前的卡方值（总误差）
    double final_chi2 = 0.0;     // 优化后的卡方值
    std::string message;         // 状态信息
};
```

**chi2（卡方）**：衡量图"不一致程度"的指标
- $\chi^2 = \sum \mathbf{e}_{ij}^T \Omega_{ij} \mathbf{e}_{ij}$
- chi2 大 → 图不一致，误差大（有漂移）
- chi2 小 → 图比较一致，误差小
- 优化目标：最小化 chi2

## $\text{代码实现}$

### 整体架构

```
loop_detector_node (独立节点)
    ├── LiDAR Iris 回环检测
    ├── GICP 验证
    ├── PoseGraph 位姿图管理
    └── g2o 优化求解

数据流：
    /keyframe_msg → callbackKeyFrame()
        → computeIrisDescriptor()       // 计算 Iris 描述子
        → addKeyFrameToPoseGraph()      // 添加节点和里程计边
        → detectLoopCandidate()         // Iris 快速筛选候选
        → verifyLoopCandidateByGicp()   // GICP/submap-to-submap几何验证
        → addLoopCandidateToPoseGraph() // 添加回环边
        → optimizePoseGraphIfNeeded()   // 触发 PGO
```

### PoseGraph 类设计

```cpp
class PoseGraph {
public:
    void configure(const PoseGraphOptions &options);
    bool addNode(const PoseGraphNode &node);
    bool addOdomEdge(uint32_t from, uint32_t to,
                     const Eigen::Isometry3d &relative_pose,
                     const Eigen::Matrix6d &information);
    bool addLoopEdge(uint32_t from, uint32_t to,
                     const Eigen::Isometry3d &relative_pose,
                     const Eigen::Matrix6d &information,
                     double score);
    PoseGraphOptimizationSummary optimize();
    
private:
    PoseGraphOptions options_;
    std::vector<PoseGraphNode> nodes_;
    std::vector<PoseGraphEdge> edges_;
    std::unordered_map<uint32_t, size_t> node_index_;
};
```

### optimize() 实现要点

```cpp
PoseGraphOptimizationSummary PoseGraph::optimize() {
    // ① 创建优化器
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    
    // ② 线性求解器 + 块求解器 + LM 算法
    auto linear_solver = std::make_unique<LinearSolverType>();
    auto block_solver = std::make_unique<BlockSolverType>(std::move(linear_solver));
    auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
    optimizer.setAlgorithm(algorithm);
    
    // ③ 添加节点
    for (const auto &node : nodes_) {
        auto *v = new g2o::VertexSE3();
        v->setId(node.id);
        v->setEstimate(node.pose);
        v->setFixed(node.fixed);  // 第一个节点6DoF全固定
        optimizer.addVertex(v);
    }
    
    // ④ 添加边
    for (const auto &edge : edges_) {
        auto *e = new g2o::EdgeSE3();
        e->setVertex(0, optimizer.vertex(edge.from_id));
        e->setVertex(1, optimizer.vertex(edge.to_id));
        e->setMeasurement(edge.relative_pose);
        e->setInformation(edge.information);
        optimizer.addEdge(e);
    }
    
    // ⑤ 执行优化
    optimizer.initializeOptimization();
    summary.initial_chi2 = optimizer.activeChi2();
    summary.iterations = optimizer.optimize(options_.max_iterations);
    
    // ⑥ 读取优化结果
    for (auto &node : nodes_) {
        auto *v = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(node.id));
        if (v) node.pose = v->estimate();
    }
}
```

### 回环检测流程

```cpp
void LoopDetectorNode::callbackKeyFrame(const KeyFrame::SharedPtr msg) {
    // ① 转换消息并计算 Iris 描述子
    LoopKeyFrame keyframe;
    convertKeyFrameMsg(*msg, keyframe);
    computeIrisDescriptor(keyframe);

    // ② 先把当前关键帧加入 PGO 图
    // 第一帧会被固定；后续帧会添加 odom edge
    const bool pgo_node_added =
        pgo_enable_ && addKeyFrameToPoseGraph(keyframe);

    // ③ Iris 快速筛选候选
    bool added_loop_edge = false;
    LoopCandidate candidate;
    if (loop_enable_ && detectLoopCandidate(keyframe, candidate)) {

        // ④ GICP 几何验证。当前实现可选 submap-to-submap，
        // 并用 odom / iris+yaw / iris-yaw 三种初值择优。
        if (!loop_gicp_enable_ || verifyLoopCandidateByGicp(keyframe, candidate)) {
            loop_candidates_.push_back(candidate);

            // ⑤ 当前节点已经在图里时，才把回环边加入 PGO
            if (pgo_node_added && pgo_enable_)
                added_loop_edge = addLoopCandidateToPoseGraph(candidate);
        }
    }

    // ⑥ 存储关键帧并触发优化
    storeKeyFrame(std::move(keyframe));
    if (pgo_enable_)
        optimizePoseGraphIfNeeded(added_loop_edge);
}
```

注意：不是只有发生回环的关键帧才有优化位姿。只要关键帧被加入PoseGraph，它就是图节点；回环边只是在已有odom chain上增加额外约束。PGO优化后，图中的所有节点都会得到优化位姿。

### GICP 验证

```cpp
bool LoopDetectorNode::verifyLoopCandidateByGicp(
    const LoopKeyFrame &current,
    LoopCandidate &candidate
) {
    // ① 找到 history 关键帧，并计算 odom 相对位姿作为基础初值
    const Eigen::Matrix4d init_guess =
        T_history_lidar.inverse() * T_current_lidar;

    // ② 可选 map-to-map：把 anchor 往前 k 帧拼成局部子图
    // 每个点云先变换到各自 anchor 的 lidar frame
    source_cloud = loop_gicp_use_submap_
        ? buildLoopGicpSubmap(current)
        : current.cloud;
    target_cloud = loop_gicp_use_submap_
        ? buildLoopGicpSubmap(history)
        : history.cloud;

    // ③ Iris 的 yaw_bias 只表示角向循环偏移，不是 x/y 位移估计。
    // 当前实现会跑三种 GICP 初值：odom、iris+yaw、iris-yaw。
    init_odom = init_guess;
    init_iris_pos = yaw(candidate.yaw_bias) * init_guess;
    init_iris_neg = yaw(-candidate.yaw_bias) * init_guess;

    result_odom = gicp(source_cloud, target_cloud, init_odom);
    result_pos  = gicp(source_cloud, target_cloud, init_iris_pos);
    result_neg  = gicp(source_cloud, target_cloud, init_iris_neg);

    // ④ 每个结果都相对自己的初值计算 correction，先过 gate
    // 再从通过 gate 的结果里选 score 最小的那个。
    best = nullptr;
    for trial in [odom, iris_pos, iris_neg]:
        correction = trial.init.inverse() * trial.result.transform;
        if trial.success &&
           trial.score < loop_gicp_score_thresh_ &&
           correction.translation_norm < loop_gicp_max_correction_trans_ &&
           correction.rotation_deg < loop_gicp_max_correction_rot_deg_:
            best = score_smaller(best, trial);

    if (!best) return false;

    candidate.gicp_score = best.score;
    candidate.source_to_target = best.transform;
    candidate.gicp_verified = true;
    return true;
}
```

这一步要注意：GICP验证的是"两片点云是否能被一个刚体变换配准好"，不等价于"它们一定是同一物理位置"。在长廊等重复结构里，GICP可能把真实存在的横向轨迹差当成配准误差修掉，所以还需要分轴gate、sequence consistency等额外约束。

### 信息矩阵计算

当前实现支持6DoF对角信息矩阵，顺序为：

```text
[x, y, z, roll, pitch, yaw]
```

示例配置：

```yaml
pgo_odom_info_diag: [150.0, 150.0, 50.0, 40.0, 40.0, 200.0]
pgo_loop_info_diag: [650.0, 650.0, 250.0, 150.0, 150.0, 850.0]
```

代码逻辑等价于：

```cpp
Eigen::Matrix6d LoopDetectorNode::makeInformationMatrix(
    const std::vector<double> &diag,
    double fallback_weight
) const {
    if (diag.size() == 6 && all_positive_finite(diag)) {
        Eigen::Matrix6d info = Eigen::Matrix6d::Zero();
        for (int i = 0; i < 6; ++i)
            info(i, i) = diag[i];
        return info;
    }

    // 兼容旧参数：数组没填或非法时退回 weight * I
    return fallback_weight * Eigen::Matrix6d::Identity();
}
```

为什么要分轴？因为不同自由度的可靠性通常不同。例如长廊场景中，水平位置和yaw可能比较重要，但z、roll、pitch容易受退化或外参误差影响。用统一的`weight * I`无法表达这种差异。

### 位姿转换工具函数

```cpp
// ROS Pose → 4×4 矩阵
Eigen::Matrix4d poseToMatrix(const geometry_msgs::msg::Pose &pose);

// 4×4 矩阵 → Isometry3d（保证旋转矩阵正交）
Eigen::Isometry3d matrixToIsometry(const Eigen::Matrix4d &matrix);

// ROS Pose → Isometry3d
Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::Pose &pose);

// Isometry3d → ROS Pose
geometry_msgs::msg::Pose isometryToPose(const Eigen::Isometry3d &pose);
```

## $\text{代码编写注意事项}$

### 1. 避免频繁拷贝点云

**问题**：GICP 配准需要传入点云，如果每次都拷贝，开销巨大

```cpp
// ❌ 错误：每次调用都拷贝点云
bool verifyLoopCandidate(const LoopKeyFrame &current, const LoopKeyFrame &history) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>(*current.cloud));
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>(*history.cloud));
    gicp_matcher_.align(src, tgt, init_guess);
}

// ✅ 正确：使用 shared_ptr，避免拷贝
bool verifyLoopCandidate(const LoopKeyFrame &current, const LoopKeyFrame &history) {
    gicp_matcher_.align(current.cloud, history->cloud, init_guess);  // 传引用
}
```

**教训**：昨晚发现 GICP 配准变慢，排查后发现是点云被频繁拷贝。改为传引用后性能提升明显。

### 2. Iris 描述子只计算一次

```cpp
// ❌ 错误：每次检测都重新计算
bool detectLoopCandidate(const LoopKeyFrame &current, ...) {
    cv::Mat1b iris_image = LidarIris::GetIris(*current.cloud);  // 重复计算
    auto descriptor = lidar_iris_->GetFeature(iris_image);
}

// ✅ 正确：存入 keyframe，只计算一次
void callbackKeyFrame(const KeyFrame::SharedPtr msg) {
    LoopKeyFrame keyframe;
    convertKeyFrameMsg(*msg, keyframe);
    computeIrisDescriptor(keyframe);  // 只计算一次
    storeKeyFrame(std::move(keyframe));
}
```

### 3. 使用 std::move 减少拷贝

**核心原则**：当你不再需要一个对象的值时，用 `std::move` 把它"转移"给别人，避免拷贝。

```cpp
// ❌ 错误：拷贝整个 keyframe（之后不再使用 keyframe）
storeKeyFrame(keyframe);

// ✅ 正确：移动语义
storeKeyFrame(std::move(keyframe));
```

**从容器"拿走"数据时，用 `std::move`**，因为原数据马上就要被销毁了，没必要拷贝一份再销毁：

```cpp
// ❌ 拷贝：取出后原数据还在容器里，浪费
auto keyframe = keyframes_.front();  // 拷贝
keyframes_.erase(keyframes_.begin()); // 再删除

// ✅ 移动：取出时直接"掏空"容器里的数据
auto keyframe = std::move(keyframes_.front());  // 移动
keyframes_.erase(keyframes_.begin());           // 删除（已经是空壳）
```

队列同理：

```cpp
// ❌ 拷贝：从队列取数据处理
while (!queue.empty()) {
    auto item = queue.front();  // 拷贝
    queue.pop();                // 删除原数据
    process(item);
}

// ✅ 移动：直接取走
while (!queue.empty()) {
    auto item = std::move(queue.front());  // 移动
    queue.pop();
    process(item);
}
```

**注意**：返回局部变量时**不要用** `std::move`，编译器会自动优化（RVO/NRVO）：

```cpp
// ❌ 错误：move 反而阻止了返回值优化
Eigen::Isometry3d matrixToIsometry(const Eigen::Matrix4d &matrix) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    return std::move(T);  // 不要这样做！
}

// ✅ 正确：直接返回
Eigen::Isometry3d matrixToIsometry(const Eigen::Matrix4d &matrix) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    return T;  // 编译器自动优化
}
```

### 4. 信息矩阵对称化

```cpp
// 添加边时自动对称化信息矩阵
bool PoseGraph::addEdge(PoseGraphEdge edge) {
    // 确保信息矩阵对称（浮点误差可能导致不对称）
    edge.information = 0.5 * (edge.information + edge.information.transpose());
    edges_.push_back(std::move(edge));
    return true;
}
```

### 5. 数值检查

```cpp
// 添加节点前检查位姿是否有效
if (!pose.matrix().allFinite()) return false;

// 添加边前检查信息矩阵是否有效
if (!information.allFinite()) return false;

// GICP 结果检查
if (!result.transform.allFinite()) return false;

// 四元数归一化
Eigen::Quaterniond q(R);
if (!q.coeffs().allFinite() || q.norm() < 1e-6) return false;
q.normalize();
```

### 6. 帧间隔过滤

```cpp
bool isCandidateAllowed(const LoopKeyFrame &current, const LoopKeyFrame &history) const {
    // ID 间隔太小 → 相邻帧，跳过
    const int id_gap = std::abs((int)current.id - (int)history.id);
    if (id_gap < loop_min_keyframe_gap_) return false;
    
    // 行驶距离太小 → 还没走远，跳过
    const double travel_gap = current.travel_distance - history.travel_distance;
    if (travel_gap < loop_min_travel_distance_) return false;
    
    return true;
}
```

### 7. 异常处理

```cpp
void LoopDetectorNode::callbackKeyFrame(const KeyFrame::SharedPtr msg) {
    try {
        // ... 主逻辑 ...
    } catch (const cv::Exception &e) {
        RCLCPP_ERROR(get_logger(), "OpenCV exception: %s", e.what());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Exception: %s", e.what());
    }
}
```

### 8. 性能监控

```cpp
// 统计各项指标
RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "Loop detector status: received=%lu stored=%lu graph_nodes=%zu "
    "graph_edges=%zu candidates=%zu iris_fail=%lu hits=%lu misses=%lu",
    received_keyframes_, stored_keyframes_,
    pose_graph_.nodeCount(), pose_graph_.edgeCount(),
    loop_candidates_.size(), iris_failures_,
    candidate_hits_, candidate_misses_);
```

### 9. 关键参数

```yaml
# 回环检测参数
loop_enable: true
loop_min_keyframe_gap: 30        # 最小帧间隔
loop_min_travel_distance: 5.0    # 最小行驶距离
loop_iris_distance_thresh: 0.30  # Iris Hamming 距离阈值
loop_history_max_current_matches: 2  # 单个history最多被最终选中次数
loop_sequence_max_history_lag_m: 5.0 # current相对history的最大单边进度滞后

# GICP 验证参数
loop_gicp_enable: true
loop_gicp_score_thresh: 0.5     # GICP 配准误差阈值
loop_gicp_max_correction_trans: 1.0  # 最大平移修正量
loop_gicp_max_correction_rot_deg: 30.0  # 最大旋转修正量

# PGO 参数
pgo_enable: true
pgo_optimize_on_loop: true       # 检测到回环时才优化
pgo_max_iterations: 20           # 最大迭代次数
pgo_odom_edge_weight: 100.0      # 里程计边权重
pgo_loop_edge_weight: 500.0      # 回环边权重
```
