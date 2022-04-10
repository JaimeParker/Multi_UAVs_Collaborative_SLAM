#  Visual SLAM

什么是SLAM？

[百度百科的定义](https://baike.baidu.com/item/SLAM/7661974)

SLAM，同步定位与建图，simultaneous localization and mapping，意义在于实现机器人的自主定位和导航

对视觉SLAM做一个初步认识

## 第一讲 前言

同时定位与地图构建，指搭载特定传感器的主体，在没有环境先验信息的情况下，于运动过程中建立环境的模型，同时估计自己的运动；使用相机作为传感器的是视觉SLAM；

《视觉SLAM十四讲》查看其代码仓库：https://github.com/gaoxiang12/slambook



## 第二讲 初识SLAM

任务如下：

* SLAM的模块及各模块的任务
* 搭建环境
* Linux下编译运行程序，调试
* cmake

只要对SLAM的流程，每个环节的大致作用了解即可，详细理论在之后掌握，先面后点；

### 2.1 引子

通过一系列连续变化的图像，进行定位与地图构建；

图像来自单目，双目，深度相机；Monocular, Stereo, RGB-D；各有优缺点；

* 双目：视差计算消耗资源，计算量主要问题
* 深度：室外难以应用

### 2.2 经典视觉SLAM框架

整个视觉SLAM的流程可以分为如下几步：

* Sensor Information，传感器信息读取
* Visual Odometry，视觉里程计，又称前端
* Optimization，后端优化
* Loop Closing，回环检测
* Mapping，建图

目前，对于静态，刚体，光照变化不明显，没有人作为干扰的场景，该SLAM已经十分成熟；（2015年左右写的）

#### 2.2.1 视觉里程计

估算相邻图像之间相机的运动，以及局部地图的样子；

* 要定量估计相机运动，必须得到相机与空间点的几何关系
* 一般是通过相邻帧之间的图像估计相机运动并恢复空间结构
* 但每次的计算都存在漂移误差，最后累计成累计漂移Accumulating Drift，需要后端优化和回环检测去解决

#### 2.2.2 后端优化

主要是处理噪声，主要是滤波与非线性优化算法

#### 2.2.3 回环检测

使机器人具有识别曾到达过场景的能力，比如判断图像之间的相似性

#### 2.2.4 建图

地图形式依据SLAM的应用场合而定，大体上分为度量地图和拓扑地图两种

**度量地图 Metric Map**

depends on its idiosyncrasy that whether it's sparse or dense, specifying the precise relative position

强调精确表示地图中物体的位置关系，用稀疏和稠密对其进行分类；有代表性的物体称为路标Landmark，对于稀疏地图，其余的不需要表示出来，地图即由路标构成；

一般定位时，稀疏地图足够；导航时，需要稠密地图；

对于二维，是许多个小格子grid；对于三维，是许多个小方块voxel；有占据，空闲，未知三种状态；

大规模度量地图出现难以避免的浪费和一致性问题；

**拓扑地图 Topological Map**

更强调元素之间的关系；

是一个Graph，由节点和边组成，只考虑节点的连通性，不考虑节点之间如何到达；

### 2.3 SLAM问题的数学表述

采集到的运动数据是离散的，由于时间尺度是离散的；

可以构建运动方程和观测方程，分别描述机器人运动中的路径和其观测到的路标；

#### 2.3.1 运动和观测

* 运动：从$k-1$​到$k$时刻，机器人的位置$x$怎样变化
* 观测：机器人在$k$时刻，于$x_k$处探测到某路标$y_j$

#### 2.3.2 运动方程和观测方程

**运动方程**
$$
x_k=f(x_{k-1},u_k,w_k)
$$

* $u_k$是运动传感器的读数，也叫输入
* $w_k$是噪声

**观测方程**
$$
z_{k,j}=h(y_j,x_k,v_{k,j})
$$

* $v_{j,k}$观测里的噪声
* $z_{k,j}$观测数据

#### 2.3.3 参数化

Parameterization，参数化方法依据传感器类型，有很多

**运动方程**

假设一个二维运动机器人，其位置position和姿态attitude由两个坐标和转角决定，即$x_k=[x,y,\theta]_k^T$；

由此，如果两个时刻之间存在线性关系，其运动方程可以具体化为：
$$
\begin{bmatrix}
x\\y\\ \theta
\end{bmatrix}_k=
\begin{bmatrix}
x\\y\\ \theta
\end{bmatrix}_{k-1}+
\begin{bmatrix}
\Delta x\\ \Delta y \\ \Delta \theta
\end{bmatrix}_k+
w_k
$$
注意，复杂情况下，运动方程有可能需要动力学分析得到；

**观测方程**

如果使用二维激光传感器，可以得到距离$r$和夹角$\phi$两个量，即$z_{k,j}=[r, \phi]^T$​；

则观测方程可以具体化为：（省略下标）
$$
\begin{bmatrix}
r\\ \phi
\end{bmatrix}=
\begin{bmatrix}
\sqrt{(x_p-x)^2+(y_p-y)^2}\\
\arctan{\cfrac{y_p-y}{x_p-x}}
\end{bmatrix}
+v
$$

#### 2.3.4 问题建模

已知运动测量的参数$u$以及传感器的读数$z$时，求解定位和建图问题（估计$x,y$）；

建模成了一个状态估计问题：如何通过带有噪声的测量数据，估计内部的，隐藏的状态变量；

对于位姿，Pose，位置+姿态；包含了旋转和平移Rotation and Translation，最多6自由度；

#### 2.3.5 系统分类

按照运动和观测方程是否线性，噪声是否服从高斯分布，分为四类：

* Linear Gaussian, namely LG, using Kalman Filter which is KF
* Non-Linear Non-Gaussian, namely NLNG, using Extened Kalman Filter which is EKF

note:

* particle filter
* graph optimization

### 2.4 编程基础



## 第三讲 三维空间刚体运动

任务：

* 三维空间刚体运动描述方式：矩阵变换，四元数，欧拉角
* Eigen库的矩阵和几何模块

### 3.1 旋转矩阵

#### 3.1.1 点和向量，坐标系

在三维坐标系中，坐标可以由一组线性空间的基和三个参数给定：
$$
a=[e_1,e_2,e_3]\begin{bmatrix}
a_1\\ a_2\\ a_3
\end{bmatrix}
$$
* $e_1$在三维时，为$3\times1$矩阵

内积：（描述向量之间投影关系）
$$
a\cdot b=a^Tb=|a||b|\cos{<a,b>}
$$
外积：
$$
a\times b=\begin{vmatrix}
i & j & k\\
a_1 & a_2 & a_3\\
b_1 & b_2 &b_3
\end{vmatrix}=
\begin{bmatrix}
a_2 b_3-a_3 b_2\\
a_3 b_1 -a_1 b_3\\
a_1 b_2 -a_2 b_1
\end{bmatrix}
$$

* 外积的结果仍然是向量，其大小为$|a||b|\sin{<a,b>}$，表示两个向量张成四边形的有向面积；
* 外积还可以表示向量的旋转；

#### 3.1.2 坐标系间的欧氏变换

两个坐标系之间的旋转变换由线性代数推导可得，由某坐标系的A‘点到A点的方程为：
$$
\begin{bmatrix}
a_1\\a_2\\a_3
\end{bmatrix}=
\begin{bmatrix}
e_1^Te_1' & e_1^Te_2' & e_1^Te_3'\\
e_2^Te_1' & e_2^Te_2' & e_2^Te_3'\\
e_3^Te_1' & e_3^Te_2' & e_3^Te_3'\\
\end{bmatrix}
\begin{bmatrix}
a_1'\\a_2'\\a_3'
\end{bmatrix}
$$
即$a=E^TE'a'$​，中间的矩阵描述了旋转，称为旋转矩阵$R$​​

由于$R$为正交矩阵Orthogonal Matrix：
$$
a'=R^{-1}a=R^Ta
$$
之后用$R$直接描述旋转，整合平移运动，有：
$$
a'=Ra+t
$$
以上用一个旋转矩阵和一个平移向量描述了欧氏空间的坐标变换关系；

#### 3.1.3 变换矩阵与齐次坐标

技巧是多引入一维向量，使之连续变化呈线性；（此时两个向量a，b为齐次坐标下的）
$$
\begin{bmatrix}
a' \\1
\end{bmatrix}=
\begin{bmatrix}
R & t\\
0 & 1
\end{bmatrix}
\begin{bmatrix}
a \\1
\end{bmatrix}=
T
\begin{bmatrix}
a \\1
\end{bmatrix}
$$
T--Transform Matrix

采用齐次坐标形式，可以写为$b=Ta$；其中，$R=E^T_b E_a$；​

### 3.2 实践Eigen

下载Eigne库

```shell
sudo apt-get install libeigen3-dev
```

要在IDE中直接调用第三方库，在自动创建的CMakeList中说明引入第三方库

```cmake
include_directories("/usr/include/eigen3")
```

you can use codes like this in CMakeList because there is only headers in Eigen3, `target_link_libraries` will be used occuring the cpps.

Also you can refer to `eigen documentation` from https://eigen.tuxfamily.org/dox-devel/modules.html

库不要求全部遍历documentation，费时费力，应该用的时候去查阅；

声明一个`double`类型的矩阵

```c++
Eigen::Matrix <double, n, m> matrix_nm;
```

不同数据类型的矩阵不能乘运算，需要显式类型转换；

矩阵转置：

```c++
matrix_nm.transpose();
```

求特征值和特征向量：

```c++
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver (matrix_33);
    cout << "Eigen values ="  << endl << eigen_solver.eigenvalues() << endl;
    cout << "Eigen vectors =" << endl << eigen_solver.eigenvectors() << endl;
```

求逆

### 3.3 旋转向量和欧拉角

#### 3.3.1 旋转向量

任意旋转都可以用一个旋转轴和一个旋转角确定；使用一个向量，其方向与旋转轴一致，而长度等于旋转角；这中向量称为**旋转向量**；

旋转向量就是李代数；

由旋转向量到旋转矩阵的过程由罗德里格斯公式表明：
$$
R=\cos{\theta}I+(1-\cos{\theta})nn^T+\sin{\theta}n^{\wedge}
$$
关于罗德里格斯公式的推导：

* [罗德里格斯公式 理解、推导](https://blog.csdn.net/q583956932/article/details/78933245)
* [罗德里格斯公式Rodrigues'Rotation Formula推导](https://zhuanlan.zhihu.com/p/113299607)

涉及李群与李代数？

#### 3.3.2 欧拉角

欧拉角把一次旋转分解成三次绕不同轴的旋转；

* 一般是yaw-pitch-roll，也就是按ZYX轴顺序的旋转
* 可以用$[r,p,y]^T$这样一个三维向量描述任意旋转

Gimbal Lock：在俯仰角为$\pm 90^{\circ}$时，第一次和第三次旋转会使用同一个轴，使得系统丢失了一个自由度，因此是奇异的

### 3.4 四元数

#### 3.4.1 四元数的定义

既是紧凑的，也没有奇异性，但不够直观，运算稍微复杂；

一个四元数$q$可以定义为：
$$
\bf{q}
\it
=q_0+q_1 i+q_2j+q_3k
$$
其中，$i,j,k$为三个虚部，满足关系式：

* $i^2=j^2=k^2=-1$
* $ij=k,ji=-k$
* $jk=i,kj=-i$
* $ki=j,ik=-j$

也可以用一个标量和一个向量来表示四元数：
$$
{\bf q}=[s,\bf{v}]
$$

* 如果虚部为0，称为虚四元数
* 如果实部为0，称为实四元数

假设某个旋转是围绕单位向量${\bf n}=[n_x,n_y,n_z]^T$进行了角度为$\theta$的旋转，这个旋转的四元数形式为：
$$
{\bf{q}}=[\cos{\frac \theta 2},n_x \sin{\frac \theta 2},n_y \sin{\frac \theta 2},n_z \sin{\frac \theta 2}]^T
$$
同理，也可以从四元数中计算出旋转轴和夹角：
$$
\left\{\begin{array}{l}\theta=2 \arccos q_{0} \\ {\left[n_{x}, n_{y}, n_{z}\right]^{T}=\left[q_{1}, q_{2}, q_{3}\right]^{T} / \sin \frac{\theta}{2}}\end{array}\right.
$$
在四元数中，任意旋转都可以用两个互为相反数的四元数表示；

* [如何形象地理解四元数](https://www.zhihu.com/question/23005815)
* [四元数讲解视频](https://www.bilibili.com/video/BV1gP4y1a7vZ?from=search&seid=10134484900592102879&spm_id_from=333.337.0.0)
* [四元数的可视化](https://www.bilibili.com/video/BV1SW411y7W1?from=search&seid=10134484900592102879&spm_id_from=333.337.0.0)

#### 3.4.2 四元数的运算

#### 3.4.3 用四元数表示旋转

假设一个三维空间点$p=[x,y,z]$，以及一个由轴角$n,\theta$指定的旋转，三维点$p$旋转后变为$p'$

首先，把三维点变为虚四元数来描述：
$$
{\bf p}=[0,x,y,z]=[0,{\bf v}]
$$
用四元数$q$表示这个轴角的旋转：
$$
{\bf q}=[\cos{\frac \theta 2}, {\bf n}\sin{\frac \theta 2}]
$$
那么，旋转后的点$p'$就可以表示为：
$$
p'=qpq^{-1}
$$

#### 3.4.4 四元数到旋转矩阵的转换

省略推导，四元数到旋转矩阵的转换方式为：
$$
R=\left[\begin{array}{ccc}1-2 q_{2}^{2}-2 q_{3}^{2} & 2 q_{1} q_{2}+2 q_{0} q_{3} & 2 q_{1} q_{3}-2 q_{0} q_{2} \\ 2 q_{1} q_{2}-2 q_{0} q_{3} & 1-2 q_{1}^{2}-2 q_{3}^{2} & 2 q_{2} q_{3}+2 q_{0} q_{1} \\ 2 q_{1} q_{3}+2 q_{0} q_{2} & 2 q_{2} q_{3}-2 q_{0} q_{1} & 1-2 q_{1}^{2}-2 q_{2}^{2}\end{array}\right]
$$
一个R表示的四元数不是唯一的；

### 3.5 相似，仿射，射影变换

#### 3.5.1 相似变换

$$
T_S=\begin{bmatrix}
sR & t\\
0^T & 1
\end{bmatrix}
$$

s为缩放因子，表示在XYZ三个坐标上进行均匀的缩放；

#### 3.5.2 仿射变换

$$
T_A=\begin{bmatrix}
A & t\\
0^T & 1
\end{bmatrix}
$$

 仿射变换只要求A是一个可逆矩阵，不必是正交矩阵；

#### 3.5.3 射影变换

射影变换是最一般的变换，矩阵形式为：
$$
T_P=\begin{bmatrix}
A & t\\
a^T & v
\end{bmatrix}
$$
在涉及相机模型之前，有大致印象即可；

### 3.6 实践：Eigen几何模块

#### 3.6.1 旋转矩阵与旋转向量

定义旋转向量`AngleAxis`，通过旋转角度和轴来定义：

```c++
Eigen::AngleAxisd rotation_vector (M_PI / 4, Eigen::Vector3d(0, 0, 1));
```

此即为绕$Z$轴旋转$\pi/4$的旋转向量；

进而可以获得旋转矩阵，使用`matrix()`方法：

```c++
Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity(); // 获得一个3x3的单位矩阵
rotation_matrix = rotation_vector.matrix();
```

此处`rotation_matrix`即为$R$，则$a'=Ra$可以表示为：

```c++
Eigen::Vector3d v （1， 0， 0）；
Eigen::Vector3d v_rotated = rotation_matrix * v;
```

#### 3.6.2 欧拉变换欧拉角

欧氏变换矩阵：

```cpp
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(rotation_matrix); // use rotation vector or matrix
    T.pretranslate(Eigen::Vector3d (1, 2, 3));
```

* `T.rorate`可以给旋转向量或旋转矩阵，均可；
* pretranslate为平移的向量；

最后用变换矩阵进行坐标变换：

```cpp
    Eigen::Vector3d v_transformed = T * v;
    cout << "v_transformed = " << v_transformed.transpose() << endl;
```

获得欧拉角：

```cpp
Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); 
```

（2，1，0）表示的是ZYX的顺序；

#### 3.6.3 四元数

```cpp
Eigen::Quaterniond q = Eigen::Quaterniond (rotation_vector);
q = Eigen::Quaterniond (rotation_matrix);
v_rotated = q * v;
```

* 可以给轴角的ector，也可以直接给旋转矩阵
* 注意旋转过后的向量在数学上和在代码中有点不同，数学上是$v'=qvq^{-1}$



## 第四讲 李群与李代数

为什么需要一个李群和李代数之间的关系？[Sophus Lie](https://en.wikipedia.org/wiki/Sophus_Lie)

参考知乎[第四讲-李群与李代数](https://zhuanlan.zhihu.com/p/33156814)

* 要解决**什么样的位姿最符合观测当前观测数据**这样的问题，是一种典型的优化问题，求解最优的$R,t$使得误差最小化；
* 由于旋转矩阵自身有约束（正交性且det=1），作为优化变量时会比较困难，因此把位姿估计变为无约束的优化问题，这就需要李群和李代数；

任务：

* 李群和李代数的概念，特殊正交群和特殊欧氏群对应的李代数的表达方式
* BCH近似公式的意义
* 李代数上的扰动模型
* Sophus对李代数进行计算

 群对加法并不封闭；

### 4.1 李群李代数基础

群对加法是不封闭的，比如特殊正交群和特殊欧氏群；

#### 4.1.1 群

**群**是一种集合加上一种运算的代数结构；

群对运算的要求有：

* 封闭性
* 结合律
* 幺元
* 逆

李群是光滑的，连续的；

#### 4.1.2 李代数的引出

$R$是某个相机的旋转，是时间的函数，$R(t)$；

推导得出，$\dot{R}(t)R(t)^T$是一个反对称矩阵；对于任意反对称矩阵，总能找到一个向量与之对应，运算符为${\vee}$；
$$
a^{\wedge}=A,A^{\vee}=a
$$
由于$\dot{R}(t)R(t)^T$是一个反对称矩阵，可以找到一个三维向量$\phi (t)$与之对应，于是有：
$$
\dot{R}(t)R(t)^T=\phi (t)^{\wedge}
$$
等式两边同乘$R(t)$，由于是正交阵，可以化为：
$$
\dot{R}(t)=\phi (t)R(t)
$$
设$t_0=0，R(0)=I$，按照导数定义，把$R(t)$在0处泰勒展开：
$$
R(t) \approx I + \phi (t_0)^{\wedge}t
$$
根据上式可以得到微分方程$\dot{R}(t)=\phi(t_0)^{\wedge}R(t)=\phi_0^{\wedge}R(t)$；解得：
$$
R(t)=\exp(\phi_0^{\wedge}t)
$$
说明在$t=0$附近，旋转矩阵和向量存在一个指数关系；这正是李群和李代数之间的指数/对数映射；

**李代数描述了李群的局部性质**；

#### 4.1.3 李代数的定义

李代数由一个集合$\mathbb{V}$，一个数域$\mathbb{F}$，和一个二元运算$[,]$组成；如满足以下几个条件，称$(\mathbb{V,F},[,])$为一个李代数，记为$\frak{g}$：

* 封闭性
* 双线性
* 自反性
* 雅可比等价

#### 4.1.4 李代数${\frak so}(3)$

${\frak so}(3)$位于${\mathbb R}^3$空间中，其元素是三维向量或三维反对称矩阵；
$$
{\frak so}(3)=\lbrace \phi \in {\mathbb R}^3, \Phi=\phi^{\wedge}\in {\mathbb R}^{3\times3} \rbrace
$$
是一个由三维向量构成的集合，每个向量对应到一个反对称矩阵，可以表达旋转矩阵的导数；

与$SO(3)$的关系由指数映射给出：
$$
R=\exp{(\phi^{\wedge})}
$$

#### 4.1.5 李代数${\frak se}(3)$

${\frak se}(3)$位于${\mathbb R}^6$空间中，其元素是六维向量；前三维为平移，后三维为旋转；

$$
{\frak se}(3)=\lbrace
\xi=\begin{bmatrix}\rho\\ \phi \end{bmatrix}\in{\mathbb R}^6,\rho \in {\mathbb R}^3,\phi\in{\frak so}(3),
\xi^{\wedge}=\begin{bmatrix}
\phi^{\wedge} & \rho\\
0^T &0\\
\end{bmatrix}\in{\mathbb R}^{4\times4}
\rbrace
$$

### 4.2 指数与对数映射

#### 4.2.1 $SO(3)$上的指数映射

$$
\exp({\theta {\bf a}^{\wedge}})=\cos{\theta {\bf I}}+(1-\cos \theta){\bf aa}^T+\sin\theta{\bf a}^{\wedge}
$$

这表明${\frak so}(3)$实际上是由旋转向量组成的空间，而指数映射即为罗德里格斯公式；

通过指数映射，把${\frak so}(3)$中的旋转向量映射到了$SO(3)$中的旋转矩阵；

同样的，对数映射可以把旋转矩阵映射到旋转向量中：
$$
\phi=\ln{R}^{\vee}=(\sum^{\infin}_{n=0}\frac{(-1)^n}{n+1}(R-I)^{n+1})^{\vee}
$$
注意，该式的推导是用迹的性质去求转角和转轴；

把旋转角固定在$\pm \pi$之间，则李群和李代数元素是一一对应的；

#### 4.2.2 $SE(3)$上的指数映射



### 4.3 李代数求导与扰动模型

#### 4.3.1 BCH公式与近似形式

李代数的加法并不能根据指数的映射关系变为矩阵的乘法；

[Wiki_BCH](https://en.wikipedia.org/wiki/Baker%E2%80%93Campbell%E2%80%93Hausdorff_formula)

根据BCH公式有近似的线性表达：

### 4.4 实践Sophus



## 第五讲 相机与图像

### 5.1 相机模型

使用针孔和畸变两个模型来描述整个投影过程；

相机的针孔模型+透镜的畸变模型，构成了相机的内参数；

#### 5.1.1 针孔相机模型

* 成像原理
* 像素坐标系的原点在左上角

可以得到由相机坐标到像素坐标的转换矩阵为，到像素坐标$[u,v]$的转换矩阵：
$$
Z\begin{pmatrix}u\\v\\1
\end{pmatrix}=
\begin{pmatrix}
f_x & 0 & c_x\\
0 & f_y & c_y\\
0 & 0 & 1\\
\end{pmatrix}
\begin{pmatrix}
X\\Y\\Z
\end{pmatrix}=
{\bf KP}
$$

* 注意，该式存在一个非齐次座标到齐次座标的转换
* $Z$为物体到光心的距离
* 称矩阵$K$为相机的内参数矩阵，*Camera Intrinsics*，可以通过**标定**的方法得到
* P是在相机坐标系下的坐标

由世界坐标到相机坐标系再到像素坐标的转换矩阵为：
$$
ZP_{uv}=Z\begin{bmatrix}u\\v\\1\end{bmatrix}={\bf K(RP_W+t)}={\bf KTP_W}
$$

* 相机的位姿$R,t$称为相机的外参数，*Camera Extrinsics*
* 可以归一化
* 丢失深度

#### 5.1.2 畸变模型

**Distortion**
$$
\begin{cases}
切向畸变\\
径向畸变 \begin{cases}桶形畸变\\ 枕形畸变\end{cases}
\end{cases}
$$
总结以下单目成像的基本过程：

* 世界坐标系下有一个固定的点P，其坐标为$P_W$
* 相机的$R,t$由变换矩阵$T$描述，则P的相机坐标为$\tilde{P_c}=RP_W+t$
* 此时$\tilde{P_c}$的分量为$[X,Y,Z]^T$，进行归一化，投影到$Z=1$平面上，$P_c=[X/Z,Y/Z,1]$
* 有畸变时，根据畸变参数计算畸变后的坐标
* 经过内参，对应到像素坐标$P_{uv}=KP_c$

#### 5.1.3 双目相机模型

#### 5.1.4 RGB-D相机模型

### 5.2 图像

### 5.3 实践：图像的存取与访问

### 5.4 实践：拼接点云



## 第六讲 非线性优化

非线性优化是状态估计问题的一个解法，也可以用扩展卡尔曼滤波的方法进行，但是在SLAM中，一般使用非线性优化的方法；

* 运动方程中的位姿可以用变换矩阵描述，使用李代数进行优化
* 观测方程使用相机成像模型；内参由相机决定，外参由位姿决定

因此，问题转化为了在噪声影响下，近似成立的运动方程和观测方程，进行精确的状态估计；

使用无约束非线性优化方法；

### 6.1 状态估计问题

#### 6.1.2 最小二乘的引出

$$
minJ(x,y)=\sum_ke_{u,k}^TR_{k}^{-1}e_{u,k}+\sum_k \sum_je_{z,k,j}^TQ_{k}^{-1}e_{z,k,j}
$$

注意，Hessian矩阵带方差；

### 6.2 非线性最小二乘

考虑一个最简单的最小二乘问题：
$$
minF(x)=\frac 12||f(x)||^2_2
$$
和二元函数极值相同：
$$
\frac{{\rm d}F}{{\rm d}x}=0
$$
只需要逐个比较导数为0处的值即可；



可以使用迭代的方法：

* 给定某个初始值$x_0$
* 对于第$k$次迭代，寻找一个增量$\Delta x_k$使得$||f(x_k+\Delta x_k)||^2_2$达到极小值
* 若$\Delta x_k$足够小，则停止
* 否则，令$x_{k+1}=x_k+\Delta x_k$，返回第二步

这使得求解导数等于0的点变成了不断寻找下降增量的问题；（机器学习，梯度下降）

#### 6.2.1 一阶和二阶梯度法

$$
F(x_k+\Delta x_k)\approx F(x_k)+J(x_k)^T\Delta x_k+ \frac12 \Delta x_k^TH(x_k)\Delta x_k
$$

* $J(x_k)$是$F(x)$关于$x$的一阶导数，也叫雅可比矩阵，梯度矩阵

#### 6.2.2 高斯牛顿法

将$f(x)$进行一阶泰勒展开：
$$
f({\bf x+\Delta x})\approx f({\bf x})+{\bf J}({\bf x})^T\Delta {\bf x}
$$
令$||f(x+\Delta x)||^2$达到最小，对该式进行求导，令导数等于0，得到如下方程组（**增量方程**）：

$$
J(x)J(x)^T\Delta x=-J(x)f(x)
$$
用$JJ^T$代替了难以求解的$H$矩阵；

**算法步骤**：

* 给定初始值$x_0$
* 对于第k次迭代，求出此时的雅可比矩阵$J(x_k)$和误差$f(x_k)$
* 求解增量方程，在这里的Hessian矩阵使用$JJ^T$近似，$H\Delta x=g$
* $\Delta x_k$足够小则停止，否则$x_{k+1}=x_k+\Delta x_k$，回到第二步

#### 6.2.3 列文伯格-马夸尔特方法

也叫信赖区域方法；Trust Region Method；

给定一个指标$\rho$来确定信赖区的范围：
$$
\rho = \frac{f(x+\Delta x)-f(x)}{J(x)^T \Delta x}
$$

* $\rho$可以反映近似的好坏；分子是实际函数下降的值，分母是近似模型下降的值；
* $\rho$接近于1，则认为近似较好
* 太小则应该缩小近似范围，太大则应该放大近似范围；

**算法步骤**：

* 给定初始值$x_0$和初始优化半径$\mu$
* 对于第k次迭代，在牛顿高斯法的基础上加入约束$||D\Delta x_k||^2<= \mu$
* 计算$\rho$
* $\rho>0.75,\mu=2\mu$，$\rho<0.25, \mu=0.5\mu$
* 若$\rho$大于某阈值，则认为近似可行，令$x_{k+1}=x_k+\Delta x_k$
* 判断算法是否收敛，不收敛则返回第二步

### 6.3 实践：曲线拟合问题

#### 6.3.1 高斯牛顿法

#### 6.3.2 LM 方法



## 参考

* 高翔，视觉SLAM十四讲
