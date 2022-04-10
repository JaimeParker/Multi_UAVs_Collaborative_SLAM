# px4uwb_ws
机载计算机获取连接的uwb标签位置，并发送给px4充当vision pose位置信息  

## 依赖ros包
请先安装好nooploop的uwb的ros驱动包，以及nooploop官方github里的serial、ros-melodic-serial*  
也请安装好mavros包  

## 使用说明
roslaunch px4_uwb swarm0.launch  
可以启动空循环uwb标签ros节点、mavros的px4节点、uwb给mavros的/mavros/vision_pose/pose话题发送x、y、z坐标的节点  
具体串口、波特率、group请在launch文件里自己改  

## 设置pixhawk
首先设置机载计算机和pixhawk的通信端口，我这里telement1（MAV_0)口给数传了，telement2口连接tx2，所以设置MAV_1_CONFIG = TELEM2。如果机载计算机连接到了pixhawk后者pixracer的usb口，则不用设置吧。  
px4固件里需要通过地面站设置EKF_AID_MASK为vision提供位置，但不提供速度和rotation角度  
EKF_HGT_MODE还是用气压计或者range finder获得高度信息  
尤其注意！uwb四个基站摆放应该满足x、y、z轴符合东北天朝向（A0-A3为x轴朝东，A0-A1为y轴朝北）。这是因为本节点直接将uwb测得的飞机x、y坐标当作local系下的东北天坐标发送到了/mavros/vision_pose/pose（mavros会自动转换成px4使用的NED坐标系），然而飞机的航向角还是根据磁罗盘确定的真正的北方，因此uwb基站的y轴必须朝北。  
当然，如果基站摆放不符合东北天坐标系，请自行修改uwb.cpp节点，将uwb的坐标旋转转换后再发给飞控.  

## 实验现象
启动launch文件后，飞机可以切换到position定点模式并定住，也可以切换到offboard模式，但是不会显示有gps信号  
