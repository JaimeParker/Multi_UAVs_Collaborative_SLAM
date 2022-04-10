# PX4 gazebo UWB 仿真方法与配置

* 主要研究：https://github.com/valentinbarral/gazebosensorplugins 方法，另一个由张一竹
* 其[说明文件](gazebosensorplugins-master/README.md)

主要需要研究如何在gazebo里配置其UWB插件，研究其UWB原理是否符合逻辑，最后如果UWB方案可行，如何与ROS，MAVROS和PX4对接。

## 1. sdf文件，model和world的搭建

### 1.1 sdf文件

sdf文件是gazebo模型（model）的组成之一，model由sdf文件和configuration文件组成；

* 其中sdf主要用标签语言构建了这一模型
* configuration文件记录了该sdf的作者，邮箱，版本等；

所以sdf文件是一个模型的主要部分，得到该作者的模型需要：

* 在`catkin`的工作空间下编译其代码
* 之后会在`devel/lib`中生成`libgtec_uwb_plugin.so`文件，需要将其拷贝到`px4/build/px4_sitl_gazebo/build_gazebo`中
* 将下面的代码添加到`px4/tools/sitl_gazebo/models/iris/iris.sdf`中，打开gazebo就可以发现iris有了这个插件

### 1.2 model

该hub主将UWB模块分为：

* tag，用于需要定位的设备上，即无人机
* anchor，用于地面基站，给其提供定位
* 两者都需要装这个插件

tag的UWB插件需要装到model上，而作者的意思，anchor则布置在world中；

To add the plugin to a Gazebo model, the next code must be present in the `.sdf` or `.urdf`.

```xml
<plugin name='libgtec_uwb_plugin' filename='libgtec_uwb_plugin.so'>
      <update_rate>25</update_rate>
      <nlosSoftWallWidth>0.25</nlosSoftWallWidth>
      <tag_z_offset>1.5</tag_z_offset>
      <tag_link>tag_0</tag_link>
      <anchor_prefix>uwb_anchor</anchor_prefix>
      <all_los>false</all_los>
      <tag_id>0</tag_id>
    </plugin>
```

* `update_rate`: num. of rates published by second. **发送频率**
* `nlosSoftWallWidth`: how thin a wall must be to let the signal go through it. **信号能通过的最大墙壁厚度**
* `tag_z_offset`: a offset in meters to add to the current height of the sensor. **tag的z轴偏移量**
* `anchor_prefix`: all the anchors placed in the scenario must have a unique name that starts with this prefix. **命名规则**
* `all_los`: if true, all the anchors are considered as in a LOS scenario. Except if there are too far from the tag, in that case they are considered as NLOS. **作者定义的几种模式，区别于是否有障碍物遮蔽**
* `tag_id`: tag identifier, a number. **tag ID**

**`tag_z_offset`**怀疑是否能应用在飞机上，需要查看其代码发现是否是定点定位；

### 1.3 world

```xml
<model name="uwb_anchor0">
      <pose>0 0 1.5 0 0 0</pose>
      <static>1</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>0.2 0.2 0.2</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
```

作者实际上提供了一个容器（盒子），插件需要装在这个盒子上，但其实只要改了模型名字，装在消防水栓上都可以；

## 2. UWB 仿真的原理剖析

![GTEC UWB Plugin in RVIZ](https://user-images.githubusercontent.com/38099967/64428790-e66b6780-d0b4-11e9-8f6f-489d8eb949c8.png)

- `LOS (Line of Sight)`: In this case there are no obstacles between the tag and the anchor. In this case, the ranging estimation is near to the actual value.
- `NLOS Soft (Non Line of Sight Soft)` : In this case, there is a thin obstacle between the tag and the anchor. The received power is lower than in the LOS case but the ranging estimation is closer to the actual value.
- `NLOS Hard`: In this case, there are too many obstacles between the tag and the anchor that the direct ray is unable to reach the receiver. But in this case, there is a path from the tag to the anchor after the signal rebounding in one wall. Thus, the estimated ranging is the corresponding to this new path, so is going to be always longer than the actual distance between the devices.
- `NLOS`: Tag and anchor are too far apart or there are too many obstacles between them, they are unable to communicate and generate a ranging estimation.

NOTE: the rebounds are only computed using obstacles placed at the same height than the tag. That means that the rebounds on the floor or the ceiling are not considered.

看了其src下几个cpp，没有发现明显的组织结构，也没发现介绍的信息，于是找了看起来最接近的cpp来分析，即[UwbPlugin.cpp](gazebosensorplugins-master/src/UwbPlugin.cpp)；

### UwbPlugin.cpp

主要函数：

* UwbPlugin，用于在world载入后加载模型的函数；
* Load，主要加载一些模型的配置参数，句柄的建立和客户端的定义，同时也定义了相关话题命名；
* OnUpdate，主要函数，应该是该cpp的主要部分；
* SetUpdateRate
* Reset

主要关注OnUpdate函数，介绍一下其测距的中心思想：

* 在gazebo中拿到tag和anchor的绝对坐标，做差得到距离；
* 使用gazebo的物理模型ray，探测是否在一组tag和anchor之间是可见的（有无遮蔽），根据不同情况选择该作者设计的不同模式进行运算；

## 3. ROS及MAVROS话题的实现，PX4的配置

This plugin publish the next topics:

- ```/gtec/gazebo/uwb/ranging/tag_id``` : where ```tag_id``` is the value configured in the plugins. This topic publish the UWB range estimations using messages of type: gtec_msgs/Ranging.msg ([https://github.com/valentinbarral/rosmsgs](https://github.com/valentinbarral/rosmsgs))

- ```/gtec/gazebo/uwb/anchors/tag_id``` : where ```tag_id``` is the value configured in the plugins. This topic publish the position of the UWB anchors in the scenario. Each anchor have a different color depending of its current LOS mode: green-LOS, yellow-NLOS Soft, blue-NLOS Hard and red-NLOS. The published messages are of type visualization_msgs/MarkerArray.msg ([visualization_msgs/MarkerArray Message](http://docs.ros.org/melodic/api/visualization_msgs/html/msg/MarkerArray.html))

