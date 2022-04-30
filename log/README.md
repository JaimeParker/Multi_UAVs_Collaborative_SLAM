# log
#### 至4.16
* `ORB_SLAM2`源码部分学习到了回环检测，从顶层了解其运行机制，暂时不用去看底层代码
* 明确了接下来的目标，[README.md stage6](../6_Map_Merging/README.md)
* 进行了初步的设计，用rosbag实现了`ORB_SLAM2`，但是仍存在很多问题
* `mp4`转`rosbag`的程序，但是有的地方需要特别注意
* [rosbag形式的ORB SLAM2实验和相机标定](../6_Map_Merging/myVideoORB.md)

#### 至4.30

* 中期报告修改（提交）
* ORB-SLAM2代码学习；[导图法](https://www.mindmeister.com/map/2257958379)；[类导图法](https://www.mindmeister.com/map/2257266424)；[知乎：函数功能主导](https://zhuanlan.zhihu.com/p/84905697)；[MapPoint](MapPoint.md)；[Frame](Frame.md)；（结束）
* [Collaborative Monocular SLAM with Multiple Micro Aerial Vehicles](../6_Map_Merging/README.md)

下周：

* 不考虑实时性（mutex），只实现PR+MM
* 弄清上文方法
* 解决PX4多机时bug， joystick
* CCM数据流分析+论文中的Map Matcher+Map Merging方法分析（不含源码）
