# Stage 1 复现多机ORB SLAM 2

根据之前的工作，难点主要在相机的添加上，如何给相机的话题前加上无人机ID的前缀，以此区别不同话题；

## launch

如何给无人机的相机加上无人机的前缀，只需要在无人机的`<group ns="uav1">`之内声明，根据`mavros`话题标准，将沿用该命名空间；

但由于`gazebo`自带的双目相机存在bug，因此删除了该模型，改用`PX4`带的，这就导致即使是在命名空间下声明，话题仍然不带无人机ID，这将导致话题重复的错误，解决方法如下：

* 再创建一个`iris_stereo_camera1`，`stereo_camera1`，注意其中的`sdf`文件名均做修改；并且在`stereo_camera1.sdf`中修改`topic name`从`stereo`为`stereo1`，这样另一架无人机的话题就是从`stereo1得到的`；
* 将`px4`的双目相机模型复制到`.ros/gazebo_models`

对于第一种，修改launch文件如下：

```xml
<!-- UAV0 -->
    <group ns="uav0">
        <!-- MAVROS and vehicle configs -->
        <arg name="ID" value="0"/>
        <arg name="fcu_url" default="udp://:14540@localhost:14580"/>
        <!-- PX4 SITL and vehicle spawn -->
        <include file="$(find px4)/launch/single_vehicle_spawn_rcs.launch">
            <arg name="x" value="0"/>
            <arg name="y" value="0"/>
            <arg name="z" value="0"/>
            <arg name="R" value="0"/>
            <arg name="P" value="0"/>
            <arg name="Y" value="0"/>
            <arg name="vehicle" value="$(arg vehicle)"/>
            
	        <arg name="my_camera" value="iris_stereo_camera"/>
            
            <arg name="mavlink_udp_port" value="14560"/>
            <arg name="mavlink_tcp_port" value="4560"/>
            <arg name="ID" value="$(arg ID)"/>
            <arg name="gst_udp_port" value="$(eval 5600 + arg('ID'))"/>
            <arg name="video_uri" value="$(eval 5600 + arg('ID'))"/>
            <arg name="mavlink_cam_udp_port" value="$(eval 14530 + arg('ID'))"/>
        </include>
        <!-- MAVROS -->
        <include file="$(find mavros)/launch/px4.launch">
            <arg name="fcu_url" value="$(arg fcu_url)"/>
            <arg name="gcs_url" value=""/>
            <arg name="tgt_system" value="$(eval 1 + arg('ID'))"/>
            <arg name="tgt_component" value="1"/>
        </include>
    </group>

    <!-- UAV1 -->
    <group ns="uav1">                              
        <!-- MAVROS and vehicle configs -->
        <arg name="ID" value="1"/>
        <arg name="fcu_url" default="udp://:14541@localhost:14581"/>
        <!-- PX4 SITL and vehicle spawn -->
        <include file="$(find px4)/launch/single_vehicle_spawn_rcs.launch">
            <arg name="x" value="2"/>
            <arg name="y" value="0"/>
            <arg name="z" value="0"/>
            <arg name="R" value="0"/>
            <arg name="P" value="0"/>
            <arg name="Y" value="0"/>
            <arg name="vehicle" value="$(arg vehicle)"/>
            
	        <arg name="my_camera" value="iris_stereo_camera1"/>
            
            <arg name="mavlink_udp_port" value="14561"/>
            <arg name="mavlink_tcp_port" value="4560"/>
            <arg name="ID" value="$(arg ID)"/>
            <arg name="gst_udp_port" value="$(eval 5600 + arg('ID'))"/>
            <arg name="video_uri" value="$(eval 5600 + arg('ID'))"/>
            <arg name="mavlink_cam_udp_port" value="$(eval 14530 + arg('ID'))"/>
        </include>
        <!-- MAVROS -->
        <include file="$(find mavros)/launch/px4.launch">
            <arg name="fcu_url" value="$(arg fcu_url)"/>
            <arg name="gcs_url" value=""/>
            <arg name="tgt_system" value="$(eval 1 + arg('ID'))"/>
            <arg name="tgt_component" value="1"/>
        </include>
    </group>
```

之后将[models](models)中的`iris_stereo_camera1`和`stereo_camera1`文件夹复制到`px4/tools/sitl_gazebo/models`下即可；

## `.sh` file

两架飞机同时用ORB，ORB接口的话题也不一样；

需要对ORB作一些修改，或者直接参考`ccm`或`covins`的方法；

需要仔细研究ORB；

## Off board control

参考之前写的多机编队控制文件；

