# XT Drone学习

内容来自[XTDrone使用文档](https://www.yuque.com/xtdrone/manual_cn)

## 仿真平台基础配置

https://www.yuque.com/xtdrone/manual_cn/basic_config

之前已经配好了ROS+PX4的环境（gazebo，marvros等），并且能实现ROS+PX4+QGC的控制起飞降落以及任务飞行；

但由于一些步骤是我之前没做的，在这里记录下来以便出错之后DEBUG；

修改`~/.bashrc`文件：

```shell
$ gedit ~/.bashrc
```

修改内容如下：

```shell
source ~/catkin_ws/devel/setup.bash
source ~/PX4_Firmware/Tools/setup_gazebo.bash ~/PX4_Firmware/ ~/PX4_Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware/Tools/sitl_gazebo
```

把相关路径改为自己的ROS工作空间和PX4路径，然后：

```shell
$ source ~/.bashrc
```

之后会提示一些修改信息：

```
GAZEBO_PLUGIN_PATH :/home/hazyparker/project/PX4-Autopilot/build/px4_sitl_default/build_gazebo
GAZEBO_MODEL_PATH :/home/hazyparker/project/PX4-Autopilot//Tools/sitl_gazebo/models
LD_LIBRARY_PATH /home/hazyparker/project/my-research/ROS_workspace/devel/lib:/opt/ros/melodic/lib:/home/hazyparker/project/PX4-Autopilot/build/px4_sitl_default/build_gazebo
```

应该是更新了一些gazebo的路径，但是我的gazebo模型都在~/.gazebo下，现在变成了PX4中的，我不确定这样会不会影响（存疑）；

结束之后运行：

```shell
$ cd ~/PX4_Firmware
$ roslaunch px4 mavros_posix_sitl.launch
```



**报错如下：**

```
[FATAL] [1641811404.988852027]: UAS: GeographicLib exception: File not readable /usr/share/GeographicLib/geoids/egm96-5.pgm | Run install_geographiclib_dataset.sh script in order to install Geoid Model dataset!
```

已经把解决方法写在报错的内容里了，但是install_geographiclib_dataset.sh我已经运行过了，而且需要安装的三个都已经存在；

```shell
hazyparker@hazy-LenovoAir14:~$ sudo /opt/ros/melodic/lib/mavros/install_geographiclib_datasets.sh
GeographicLib geoids dataset egm96-5 already exists, skipping
GeographicLib gravity dataset egm96 already exists, skipping
GeographicLib magnetic dataset emm2015 already exists, skipping
```

```
REQUIRED process [mavros-6] has died!
process has finished cleanly
log file: /home/hazyparker/.ros/log/22a2f8d0-7202-11ec-914c-0c7a150d74d3/mavros-6*.log
Initiating shutdown!
```

但是报错的原因是文件不可读，考虑一下修改文件权限；

结果/usr/share/GeographicLib/geoids/egm96-5.pgm这个文件根本没找到；为什么,sh文件能找到那个文件夹打开却是空的暂时不知道，但是解决方法是，在[Link](https://gitee.com/pi-lab/research_cluster_navigation/tree/master/tools/ros_gazebo_install_scripts/gazebo/GeographicLib)这里可以找到那三个文件，直接复制到 /usr/share/GeographicLib就可以：

```shell
$ sudo cp -r geoids/ gravity/ magnetic//usr/share/GeographicLib/
```

至此，该BUG解决；（但是原因仍然未知）；



## 视觉SLAM

https://www.yuque.com/xtdrone/manual_cn/vslam

