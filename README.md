# Collaborative SLAM Functioned on Multi UAVs

项目来自`gitee`，[click link](https://gitee.com/hazyparker/my-research)

## 0. 文件索引

### 0.1 第一阶段 research plan

* [研究计划](0_research_plan/research_plan.md)
* [开题报告](0_research_plan/thesis_proposal.md)
* [文献翻译](0_research_plan/papers.md)

### 0.2 第二阶段 ROS

* [任务实现](1_basic/README.md)

### 0.3 第三阶段 SLAM

* [README](2_SLAM/README.md)

### 0.4 第四阶段 On board SLAM

* [README](3_Onboard_SLAM/README.md)

### 0.5 第五阶段 Multi SLAM

* [README](4_Multi_SLAM/README.md)

### 0.6 第六阶段 Map Merge

* [README](6_Map_Merging/README.md)

### 0.7 第七阶段 Paper

* [Outline](7_Paper/README.md)
* [Draft](7_Paper/draft.md)
* [pdf论文（可能太大，加载失败）](7_Paper/LaTex/main.pdf)
* [压缩版pdf论文](7_Paper/thesis_compressed.pdf)

## 1. 研究目标

本研究旨在实现一套能够在室内高精度环境或GPS拒止环境下使用视觉进行多机定位和大范围建图的多无人机协同SLAM的方案；其中：

* 在SLAM方面：掌握一些优秀的开源方案，选择各自优点做出一定的融合，并且有一套针对地图融合的方法。
* 在仿真方面：在ROS的gazebo仿真平台中实现一定的集群控制方法，能够控制多个无人机协同完成同时定位与建图的任务。
* 在真机方面：实现单机的视觉SLAM；在安全的前提下实现双机协同SLAM，将仿真环境下的协同SLAM算法在真机上完成验证，得到场景地图。

## 2. 主要思路

多机协同SLAM能大大提高任务进行的效率，但同时由于无人机数量较多，协同上存在一定困难；UWB的引入可以提高定位精度。因此，本次研究内容是多机协作进行定位与建图，并且可以引入UWB提高精度。

研究内容分为三个模块：SLAM模块，仿真模块和真机模块。

SLAM模块的主要内容是实现一套可协作的SLAM方案，实现的步骤有：

1. 研究传统的视觉SLAM的特征点提取、匹配、初始化、后端优化等技术，研究机器人的位姿估计技术；研究并了解SLAM技术的整体框架
2. 研究CCM-SLAM方案，重点研究其协同的机制和方法，服务端到子端的信息传递和接口设计等
3. 研究VINS-Fusion方案中的VIO方法，研究如何利用IMU与相机数据联合进行更加准确的位姿估计

仿真模块的主要内容是在ROS的gazebo中研究如何实现多机协同的同时定位与建图，实现的步骤有：

1. 首先研究PX4和MAVROS之间的通信方式，ROS的话题发布和订阅方式，研究如何用程序解锁一架无人机、使其进入Offboard模式、起飞悬停并降落
2. 研究如何用程序发布话题，控制无人机按照航路点飞行
3. 研究如何构建多机的仿真环境，如何对多机进行控制，其控制策略的选择，即集中式或分布式的多机编队控制
4. 研究如何更改无人机的定位设置，将其从GPS定位改为视觉SLAM定位；并且完成单机的摄像头内容读取
5. 研究如何在gazebo中载入其他场景，在场景中控制无人机飞行，并且对拍摄到的画面进行建图，完成自身定位
6. 研究如何在gazebo中完成多机基于视觉的同时定位与建图，并且拼合地图，用第三方软件显示；研究多机的联合优化与协同方法

真机模块的主要内容是控制无人机的协同飞行及通信，实现的步骤有：

1. 研究无人机通过MAVROS，MAVLINK与地面站的通信方法，尤其是用于SLAM的关键数据的传输
2. 研究多无人机与地面站之间的、多无人机之间的数据传输
3. 研究多无人机之间的可变基线控制技术，如何设计一个详细的算法控制基线距离

## 3. 关键技术

1. ROS和PX4的使用：ROS（Robotics Operating System）作为最完善的开源机器人操作系统，通过其MAVROS发布和订阅的话题机制，能够和PX4软件建立起通信，进而控制无人机；ROS可以管理各种功能包，而SLAM的实现正需要借助大量的功能包，无人机的控制也需要功能包中的具体程序；ROS中的gazebo为算法和控制方法提供了一个很好的仿真平台。
2. 多终端的同时定位与建图：要获得最终的场景地图和各终端的定位信息，必须有一套能够根据各个终端不同的视觉数据信息、各自构建的地图进行重叠检测、局部剔除与增强，进而才能获得完整的拼合后的地图以及各终端的具体位置。
3. 多无人机协作的框架和信息传递：多无人机完成协作任务必须基于一个具体的框架，该框架需要分配各机在任务中的工作，需要制定各机与地面站的通信方法、各机传输数据的内容；由于多机同时图传对带宽的占用较大，可以协调传输或在各机上做一定处理后传输，传输的内容可以是关键帧、建立的地图点等数据。
4. 真机的实验：最终的实验需要使用真机完成，因此必须掌握真机的操作、与地面站之间的通信方法、使用ROS功能包中的SLAM程序的方法，进而完成真机飞行中各机图像的采集、处理，各机的定位和地图的构建。

## 4. 研究计划

2022/1/15-2021/1/31：学习SLAM算法，了解其基本原理

2022/1/31-2022/2/14：跑通无人机上的SLAM代码，能够使用无人机建图

2022/2/14-2022/2/28：详细学习OpenCV的成员函数

2022/3/1-2022/3/21：研究掌握拼接地图的策略和方法

2022/3/21-2022/4/7：研究多机SLAM，并且引入地图拼接

2022/4/7-2022/5/1：整合代码，真机实验

2022/5/1-2022/6/1：论文撰写，准备答辩

剩余6周安排；

* week1 完成ORB的代码
* week2 自己写地图拼，做普通场景的测试
* week3 融合，仿真
* week4 上机
* week5 论文

## 5. 参考资料

* [毕业设计指南](https://gitee.com/pi-lab/graduation_project_guide)
* [如何写研究计划LN](https://gitee.com/pi-lab/senior-design-2021/blob/master/ReasearchPlanWritting.md)，[如何写研究计划BSH](https://gitee.com/pi-lab/pilab_research_fields/blob/master/tips/master_life/Guidelines_for_ResearchPlanning.md)
* [学士论文LaTex模板与示例](https://gitee.com/pi-lab/template_bachelor)
* [如何做研究](https://gitee.com/pi-lab/pilab_research_fields/blob/master/tips/HowToResearch.md)
* [参考差异检测的研究计划](https://gitee.com/pi-lab/research_change_detection)
