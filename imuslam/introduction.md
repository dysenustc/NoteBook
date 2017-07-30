# IMU+SLAM 方案调研——典型方案
##### 导言
> IMU+SLAM / VIO 方案按照采用的优化方式可以分为滤波器方式和优化方式 (filter-baseed or optimization-based)，其中 optimization-based 也可称为 keyframe-based。按照是否把图像特征信息加入状态向量进行分类，可以分为松耦合(loosely-coupled)和紧耦合(tightly-coupled)，这是两种独立的分类方法，首先看是基于滤波还是优化的，然后进一步根据状态向量中是否加入了图像的特征信息来判断松紧耦合。
 __VIO主要尝试的是融合 Viusal 和 IMU 的信息，因此后面的论述中也就主要考虑这两种数据。__

 ***

## 1. Filter-baseed
### 1.1 紧耦合 (Tightly-coupled) 概念
&emsp;&emsp;紧耦合需要将图像特征加入到特征向量中，后续的所有操作都是针对整个特征向量的，因此整个系统状态向量的维度会很高，计算量相对较大。
<div align=center>
<img src=".//tightly coupled.png" alt="Tightly-coupled"/>
</div>

### 1.2 紧耦合实现方案
&emsp;&emsp;紧耦合比较经典的算法是MSCKF，ROVIO。MSCKF最初由 Anastasios Mourikis 教授2007年发表于 IEEE Robotics and Automation，之后2013年其学生  Mingyang Li 2013年发表了改进版，进一步提高了 MSCKF 的定位精度。有专利保护，暂时没有开源。据说 google tango 里面使用的也是这种算法。ROVIO 由 ETHZ_ASL(苏黎世联邦理工学院自主系统实验室) 实验室2015年发表于 IEEE Intelligent Robots and Systems (IROS)，同时开源了相关算法，算法基于 ROS 框架实现，同时也提供的相关数据集。

#### 1.2.1 MSCKF [1.0](http://www.ee.ucr.edu/~mourikis/papers/MourikisRoumeliotis-ICRA07.pdf) [2.0](http://www.ee.ucr.edu/~mourikis/papers/Li2013IJRR.pdf) (A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation)
&emsp;&emsp;传统的 EKF-based SLAM 做 IMU 融合时，一般是每个时刻的 state vector 保存当前的 pose、velocity、以及 3D map points 坐标等（IMU 融合时一般还会加入 IMU 的 bias），然后用 IMU 做 predict step，再用 image frame 中观测 3D map points 的观测误差做 update step。MSCKF 的 motivation 是，EKF 的每次 update step 是基于 3D map points 在单帧 frame 里观测的，如果能基于其在多帧中的观测效果应该会好（有点类似于 local bundle adjustment 的思想）。所以 MSCKF 的改进如下：predict step 跟 EKF 一样，但是将 update step 推迟到某一个 3D map point 在多个 frame 中观测之后进行计算，在 update 之前每接收到一个 frame，只是将 state vector 扩充并加入当前 frame 的 pose estimate。这个思想基本类似于 local bundle adjustment（或者 sliding window smoothing），在update step时，相当于基于多次观测同时优化 pose 和 3D map point。具体细节可以参考paper。

&emsp;&emsp;MSCKF 维护一个 pose 的时间序列的 FIFO，也可称为滑动窗口(silde window) *这种做法在其他基于优化的方案中也得到了广泛的应用，* 一个特征点在在滑动窗口的几个位姿都被观察到的话，就会在这几个位姿间建立约束，从而进行 keyframe 的更新。
<div align=center>
<img src=".//EKF-MSCKF.png" alt="Tightly-coupled"/>
</div>

#### 1.2.2 ROVIO [1](https://github.com/ethz-asl/rovio)  [2](https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/155340/eth-48374-01.pdf)(Robust visual inertial odometry using a direct EKF-based approach)
&emsp;&emsp;ROVIO 的基本原理是






 <style>blockquote,p{margin-left:2em}</style>
