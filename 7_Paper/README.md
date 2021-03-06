# Paper

论文大纲；

==多机或者集群的内容可以多写一些==

## 摘要

## 第一章 绪论

也可以叫介绍

### 1.1 研究背景

### 1.2 研究内容及论文结构

## 第二章 ROS与PX4介绍

介绍ROS和PX4在本研究中的作用；简写；

### 2.1 ROS

ROS介绍

#### 2.1.1 ROS的消息机制

介绍ROS核心的消息机制，及使用IDE的简单调试过程

#### 2.1.2 gazebo仿真

主要介绍gazebo仿真的机制，world，model，sdf的层级关系

### 2.2 PX4 Auto Pilot

PX4介绍

#### 2.2.1 Fail Safe机制

主要写第一次由于Joystick导致的PX4无法启动问题，及其背后的安全生效机制；

#### 2.2.2 EKF与飞行模式

主要写第二次由于home position导致的无法起飞问题，及背后的EKF机制；

#### 2.2.3 联合MAVROS的Off-board模式

主要写单机的飞行和多机的编队飞行控制，及背后的off-board模式；

## 第三章 SLAM系统设计

SLAM的基本介绍；本章内容的基本介绍（本章的逻辑）；

### 3.1 SLAM系统

#### 3.1.1 SLAM的分类

简单介绍视觉，激光，语义的内容；

#### 3.1.2 相机的参数及成像原理

成像原理，内外参矩阵，相机标定；

#### 3.1.3 视觉SLAM的基本步骤

视觉里程计，后端优化，回环检测，建图；

### 3.2 ORB-SLAM2

简单介绍一下CCM-SLAM；

#### 3.2.1 ORB特征点及描述子

介绍ORB特色的特征点和描述子；

#### 3.2.2 ORB-SLAM2的主要进程

Tracking，LoopClosing，LocalMapping等

#### 3.2.3 

### 3.3 CCM-SLAM

简单介绍一下CCM-SLAM；

#### 3.3.1 CCM-SLAM的结构

介绍CCM的架构，其实是和ORB类似的，着重介绍一下其通信方面；

#### 3.3.2 Client与Server机制

这里详细分析一下其通信的方法；

### 3.4 多机协同及地图融合方案

#### 3.4.1 算法原理

介绍地图融合的算法原理，框图加文字形式；

#### 3.4.2 编程实现

## 第四章 多无人机SLAM仿真

介绍章节逻辑；

### 4.1 gazebo仿真环境配置

介绍仿真过程中gazebo环境的配制方法；

#### 4.1.1 场景

自建场景的方法；相关参数的设置；

#### 4.1.2 launch文件

launch文件的意义；

launch文件的修改方法；

### 4.2 单机SLAM仿真

#### 4.2.1 EKF设置及启动仿真

EKF的相关设置方法；

#### 4.2.2 视觉定位的坐标变换

从视觉定位坐标到MAVROS坐标的转换方法；

在这里需要附上仿真结果；

### 4.3 多机SLAM仿真

不写三级标题了，直接写内容

## 第五章 实验与评估

### 5.1 真机实验

### 5.2 评估

## 第六章 结论与展望

### 6.1 结论

### 6.2 展望

## 参考文献

## 致谢
