# **IMU+SLAM 方案调研——典型方案**
##### 导言
>&emsp;&emsp;IMU+SLAM / VIO 方案按照采用的优化方式可以分为滤波器方式和优化方式 (filter-baseed or optimization-based)，其中 optimization-based 也可称为 keyframe-based。按照是否把图像特征信息加入状态向量进行分类，可以分为松耦合(loosely-coupled)和紧耦合(tightly-coupled)，这是两种独立的分类方法，首先看是基于滤波还是优化的，然后进一步根据状态向量中是否加入了图像的特征信息来判断松紧耦合。
 __VIO主要尝试的是融合 Viusal 和 IMU 的信息，因此后面的论述中也就主要考虑这两种数据。__

 ***

[TOC]
## 1. 基于滤波器的方案(Filter-baseed)
### 1.1 紧耦合 (Tightly-coupled) 概念
&emsp;&emsp;紧耦合需要将图像特征加入到特征向量中，后续的所有操作都是针对整个特征向量的，因此整个系统状态向量的维度会很高，计算量相对较大。
<div align=center>
<img src=".//tightly coupled.png" alt="Tightly-coupled"/>
</div>

### 1.2 紧耦合实现方案
&emsp;&emsp;紧耦合比较经典的算法是 MSCKF，ROVIO。MSCKF 最初由 Anastasios Mourikis 教授2007年发表于 IEEE Robotics and Automation，2013年其学生  Mingyang Li 发表了改进版，进一步提高了 MSCKF 的定位精度，MSCKF 有专利保护，暂时没有开源。据说 google tango 里面使用的也是这种算法。ROVIO 由 ETHZ_ASL(苏黎世联邦理工学院自主系统实验室) 实验室2015年发表于 IEEE Intelligent Robots and Systems (IROS)，同时开源了相关算法，算法基于 ROS 框架实现，同时也提供的相关数据集。

#### 1.2.1 MSCKF [<small>**Paper 1.0**</small>](http://www.ee.ucr.edu/~mourikis/papers/MourikisRoumeliotis-ICRA07.pdf) [<small>**Paper 2.0**</small>](http://www.ee.ucr.edu/~mourikis/papers/Li2013IJRR.pdf) (A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation)
&emsp;&emsp;传统的 EKF-based SLAM 做 IMU 融合时，一般是每个时刻的 state vector 保存当前的 pose、velocity、以及 3D map points 坐标等（IMU 融合时一般还会加入 IMU 的 bias），然后用 IMU 做 predict step，再用 image frame 中观测 3D map points 的观测误差做 update step。MSCKF 的 motivation 是，EKF 的每次 update step 是基于 3D map points 在单帧 frame 里观测的，如果能基于其在多帧中的观测效果应该会好（有点类似于 local bundle adjustment 的思想）。具体细节可以参考paper。

&emsp;&emsp;传统的EKF-SLAM框架中，特征点的信息会加入到特征向量和协方差矩阵里,这种方法的缺点是特征点的信息会给一个初始深度和初始协方差，如果不正确的话，极容易导致后面不收敛，出现 inconsistent 的情况。MSCKF 维护一个 pose 的时间序列的 FIFO，predict step 跟 EKF 一样，但是将 update step 推迟到某一个 3D map point 在多个 frame 中观测之后进行计算，在 update 之前每接收到一个 frame，只是将 state vector 扩充并加入当前 frame 的 pose estimate。这个思想基本类似于 local bundle adjustment（或者 sliding window smoothing），在update step时，相当于基于多次观测同时优化 pose 和 3D map point。 一个特征点在在滑动窗口的几个位姿都被观察到的话，就会在这几个位姿间建立约束，从而进行 keyframe 的更新。
<div align=center>
<img src=".//EKF-MSCKF.png" alt="Tightly-coupled"/>
</div>

#### 1.2.2 ROVIO [<small>**Code**</small>](https://github.com/ethz-asl/rovio)  [<small>**Paper**</small>](https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/155340/eth-48374-01.pdf)(Robust visual inertial odometry using a direct EKF-based approach)
&emsp;&emsp;ROVIO 的基本原理是

### 1.3 松耦合实现方案
&emsp;&emsp;松耦合不把图像的特征加入状态向量，而是把图像作为一个独立的数据源，计算出 Visual Odometry 之后再和 IMU 数据进行融合。ETHZ 的 Stephan Weiss 在这方面做了很多的研究，他提出的 SSF 和 MSF 都是这方面比较优秀的开源算法。
<div align=center>
<img src=".//loosely coupled.png" alt="Loosely-coupled"/>
</div>

#### 1.3.1 SSF [<small>**Code**</small>](https://link.zhihu.com/?target=https%3A//github.com/ethz-asl/ethzasl_sensor_fusion) [<small>**Paper**</small>](https://www.research-collection.ethz.ch/handle/20.500.11850/52698)
*这篇文章因版权问题暂未下载。*

&emsp;&emsp;SSF采用一个24维的状态向量描述系统

<div align="center">
<img src="http://chart.googleapis.com/chart?cht=tx&chl=\Large X=\{p_w^{i^T} \quad v_w^{i^T} \quad q_w^{i^T} \quad b_w^T \quad b_a^T \quad \lambda \quad p_i^c \quad q_i^c\}" style="border:none;">
</div>

&emsp;&emsp;代码中 SSF_core 主要处理 state 的数据，里面有预测和更新两个过程。SSF_update 则主要处理另外一个传感器的数据，主要生成测量的过程。
<div align=center>
<img src=".//SSF_coreandupdate.png" alt="Loosely-coupled"/>
</div>

## 2. (基于关键帧优化的方案)Optimization-based
&emsp;&emsp;随着研究的不算深入和计算平台性能的不断提升，基于优化的 VIO 也取得了不错的效果。其中主要研究方向都是基于紧耦合实现的，典型的有 OKVIS、 HKUST-Aerial-Robotics最近也开源了其开发的算法。

### 2.1 紧耦合实现方案

#### 2.1.1 OKVIS [<small>**Code**</small>](https://github.com/ethz-asl/okvis) [<small>**Paper**</small>](https://spiral.imperial.ac.uk/bitstream/10044/1/23413/2/ijrr2014_revision_1.pdf)(Keyframe-Based Visual-Inertial Odometry Using Nonlinear Optimization
)

&emsp;&emsp;相对应于 MSCKF 和 ROVIO 的 filter-based SLAM，OKVIS 是 keyframe-based SLAM做 visual-inertial sensor fusion 的代表。从 MSCKF 的思想基本可以猜出，OKVIS 是将 Visual 观测和 IMU 观测显式 formulate 成优化问题，一起去优化求解 pose 和 3D map point。的确如此，OKVIS 的优化目标函数包括一个 reprojection error term 和一个 imu integration error term，其中已知的观测数据是每两帧之间的 feature matching 以及这两帧之间的所有 IMU采样数据的积分(注意: IMU 采样频率一般高于视频帧率)，待求的是 camera pose 和 3D map point，优化针对的是一个 bounded window 内的 frames (包括最近的几个 frames 和几个 keyframes)。
<div align=center>
<img src=".//OKVIS.png" alt="Loosely-coupled"/>
</div>
&emsp;&emsp;上图左边是纯视觉的 Odemorty,右边是视觉 IMU 融合的 Odemorty 结构， 这个核心在于 Frame 通过 IMU 进行了联合，但是 IMU 自身测量有一个随机游走的偏置， 所以每一次测量又通过这个偏置联合在了一起， 形成了右边那个结构，对于这个新的结构， 我们需要建立一个统一的损失函数进行联合优化。OKVIS 的优化函数构造如下，左边是视觉数据误差项，右边是 IMU 数据误差项。
<div align=center>
<img src=".//OKVISimu.png" alt="Loosely-coupled"/>
</div>

#### 2.1.2 VINS-Mobile [<small>**Code**</small>](https://github.com/HKUST-Aerial-Robotics/VINS-Mobile) [<small>**Paper**</small>](http://www.ece.ust.hk/~eeshaojie/ismar2017peiliang.pdf) Monocular Visual-Inertial State Estimator on Mobile Phones

&emsp;&emsp;VINS-Mobile 是香港科技大学空中机器人实验室(搜索结果
HKUST Aerial Robotics Group)的研究人员开发的单目视觉惯性姿态估计系统。这个系统可以运行在iOS设备上，为手机端的增强现实应用提供精确的定位功能。同时该系统也在应用在了无人机控制上，并取得了很好的效果。并在2017年5月份开源了算法源码。

&emsp;&emsp;VINS-Mobile使用滑动窗口优化方法，采用视觉和IMU融合的方法，提供了高精度的定位。VINS-Mobile的初始化是全自动的，并带有闭环检测模块，累计误差通过全局POSE GRAPH SLAM得到实时校正。

&emsp;&emsp;在这里简单介绍一下demo里面的流程。程序一启动会自动初始化，计算出手机的初始位姿, 速度, 陀螺仪的偏置, 和三维空间中的特征。之后随着手机的移动，VINS 会显示手机运动的实时轨迹。
<div align=center>
<img src=".//VINSDEMOVINS.png" alt="Loosely-coupled"/>
</div>

&emsp;&emsp;在屏幕中点击AR按钮会出现AR interaction，当手机在三维空间中检测出平面时会显示一张网格，这个时候点击网格会放置一个立方体，立方体的尺寸是根据平面点的平均深度计算的。完整的视频DEMO [地址。](https://youtu.be/qazzGT84Scc)
<div align=center>
<img src=".//VINSDEMOAR.png" alt="Loosely-coupled"/>
</div>
