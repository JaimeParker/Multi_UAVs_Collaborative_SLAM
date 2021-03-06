# 比较launch文件的区别

## `XTDrone`

```xml
<?xml version="1.0"?>
<launch>
    <!-- MAVROS posix SITL environment launch script -->
    <!-- launches Gazebo environment and 2x: MAVROS, PX4 SITL, and spawns vehicle -->
    <!-- vehicle model and world -->
    <arg name="est" default="ekf2"/>
    <arg name="world" default="$(find mavlink_sitl_gazebo)/worlds/indoor1.world"/>
    <!-- gazebo configs -->
    <arg name="gui" default="true"/>
    <arg name="debug" default="false"/>
    <arg name="verbose" default="false"/>
    <arg name="paused" default="false"/>
    <!-- Gazebo sim -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="gui" value="$(arg gui)"/>
        <arg name="world_name" value="$(arg world)"/>
        <arg name="debug" value="$(arg debug)"/>
        <arg name="verbose" value="$(arg verbose)"/>
        <arg name="paused" value="$(arg paused)"/>
    </include>
     <!-- iris_0 -->
     <group ns="iris_0">
        <!-- MAVROS and vehicle configs -->
            <arg name="ID" value="0"/>
            <arg name="ID_in_group" value="0"/>
            <arg name="fcu_url" default="udp://:24540@localhost:34580"/>
        <!-- PX4 SITL and vehicle spawn -->
        <include file="$(find px4)/launch/single_vehicle_spawn_xtd.launch">
            <arg name="x" value="0"/>
            <arg name="y" default="7.5"/>
            <arg name="z" default="1"/>
            <arg name="R" value="0"/>
            <arg name="P" value="0"/>
            <arg name="Y" value="0"/>
            <arg name="vehicle" value="iris"/>
            <arg name="sdf" value="iris_stereo_camera"/>
            <arg name="mavlink_udp_port" value="18570"/>
            <arg name="mavlink_tcp_port" value="4560"/>
            <arg name="ID" value="$(arg ID)"/>
            <arg name="ID_in_group" value="$(arg ID_in_group)"/>
        </include>
        <!-- MAVROS -->
        <include file="$(find mavros)/launch/px4.launch">
            <arg name="fcu_url" value="$(arg fcu_url)"/>
            <arg name="gcs_url" value=""/>
            <arg name="tgt_system" value="$(eval 1 + arg('ID'))"/>
            <arg name="tgt_component" value="1"/>
        </include>
    </group>
</launch>
<!--the launch file is generated by XTDrone multi-vehicle generator.py  -->
```

## Mine

```xml
<?xml version="1.0"?>
<launch>
    <!-- MAVROS posix SITL environment launch script -->
    <!-- launches MAVROS, PX4 SITL, Gazebo environment, and spawns vehicle -->
    <!-- vehicle pose -->
    <arg name="x" default="0"/>
    <arg name="y" default="0"/>
    <arg name="z" default="0"/>
    <arg name="R" default="0"/>
    <arg name="P" default="0"/>
    <arg name="Y" default="0"/>

    <!-- vehicle model and world -->
    <arg name="est" default="ekf2"/>
    <arg name="vehicle" default="iris"/>
    <!-- add stereo camera for iris -->
    <arg name="my_camera" default="iris_stereo_camera"/>
    <arg name="world" default="$(find mavlink_sitl_gazebo)/worlds/empty.world"/>
    <!-- also need to revise sdf -->
    <arg name="sdf" default="$(find mavlink_sitl_gazebo)/models/$(arg my_camera)/$(arg my_camera).sdf"/>
    <!-- <arg name="sdf" default="$(find mavlink_sitl_gazebo)/models/$(arg vehicle)/$(arg vehicle).sdf"/> -->

    <!-- gazebo configs -->
    <arg name="gui" default="true"/>
    <arg name="debug" default="false"/>
    <arg name="verbose" default="false"/>
    <arg name="paused" default="false"/>
    <arg name="respawn_gazebo" default="false"/>
    <!-- MAVROS configs -->
    <arg name="fcu_url" default="udp://:14540@localhost:14557"/>
    <arg name="respawn_mavros" default="false"/>
    <!-- PX4 configs -->
    <arg name="interactive" default="true"/>
    
    <!-- PX4 SITL and Gazebo -->
    <!-- change to new world -->
    <include file="$(find px4)/launch/posix_sitl_newWorld.launch">
        <arg name="x" value="$(arg x)"/>
        <arg name="y" value="$(arg y)"/>
        <arg name="z" value="$(arg z)"/>
        <arg name="R" value="$(arg R)"/>
        <arg name="P" value="$(arg P)"/>
        <arg name="Y" value="$(arg Y)"/>
        <arg name="world" value="$(arg world)"/>
        <arg name="vehicle" value="$(arg vehicle)"/>
        <arg name="sdf" value="$(arg sdf)"/>
        <arg name="gui" value="$(arg gui)"/>
        <arg name="interactive" value="$(arg interactive)"/>
        <arg name="debug" value="$(arg debug)"/>
        <arg name="verbose" value="$(arg verbose)"/>
        <arg name="paused" value="$(arg paused)"/>
        <arg name="respawn_gazebo" value="$(arg respawn_gazebo)"/>
    </include>
    <!-- MAVROS -->
    <include file="$(find mavros)/launch/px4.launch">
        <!-- GCS link is provided by SITL -->
        <arg name="gcs_url" value=""/>
        <arg name="fcu_url" value="$(arg fcu_url)"/>
        <arg name="respawn_mavros" value="$(arg respawn_mavros)"/>
    </include>
</launch>
```

## Analysis

1. `<!-- vehicle model and world -->` 里面就直接把world加载了，而我是在后面SITL里面修改的，前面没修改
2. `<!-- Gazebo sim -->` 里面仍然是empty world，但这一部分我的里面没有
3. `<!-- MAVROS configs -->`中端口号不一样
4. `<!-- PX4 SITL and vehicle spawn -->`中设置了udp和tcp端口，和之前的还不一致
5. `<!-- PX4 SITL and vehicle spawn -->`，继续比对其launch文件

通过对比发现，`XTDrone`的launch文件中，关于PX4软件在环仿真和gazebo用的launch是spawn的launch，虽然和`posix_sitl`差不多，但是spawn更倾向于多无人机设计。

### `XTdrone`

```xml
<?xml version="1.0"?>
<launch>
    <!-- Posix SITL environment launch script -->
    <!-- launchs PX4 SITL and spawns vehicle -->
    <!-- vehicle pose -->
    <arg name="x" default="0"/>
    <arg name="y" default="0"/>
    <arg name="z" default="0"/>
    <arg name="R" default="0"/>
    <arg name="P" default="0"/>
    <arg name="Y" default="0"/>
    <!-- vehcile model and config -->
    <arg name="sdf" default="plane"/>
    <arg name="est" default="ekf2"/>
    <arg name="vehicle" default="plane"/>
    <arg name="ID" default="0"/>
    <arg name="ID_in_group" default="0"/>
    <env name="PX4_SIM_MODEL" value="$(arg vehicle)" />
    <env name="PX4_ESTIMATOR" value="$(arg est)" />
    <arg name="mavlink_udp_port" default="14560"/>
    <arg name="mavlink_tcp_port" default="4560"/>
    <!-- PX4 configs -->
    <arg name="interactive" default="true"/>
    <!-- generate sdf vehicle model -->
    <arg name="cmd" default="xmlstarlet ed -d '//plugin[@name=&quot;mavlink_interface&quot;]/mavlink_tcp_port' -s '//plugin[@name=&quot;mavlink_interface&quot;]' -t elem -n mavlink_tcp_port -v $(arg mavlink_tcp_port) $(find px4)/Tools/sitl_gazebo/models/$(arg sdf)/$(arg sdf).sdf"/>
    <param command="$(arg cmd)" name="model_description"/>
    <!-- PX4 SITL -->
    <arg unless="$(arg interactive)" name="px4_command_arg1" value=""/>
    <arg     if="$(arg interactive)" name="px4_command_arg1" value="-d"/>
    <node name="sitl_$(arg ID)" pkg="px4" type="px4" output="screen" args="$(find px4)/ROMFS/px4fmu_common -s etc/init.d-posix/rcS -i $(arg ID) -w sitl_$(arg vehicle)_$(arg ID) $(arg px4_command_arg1)">
    </node>
    <!-- spawn vehicle -->
    <node name="$(arg vehicle)_$(arg ID)_spawn" output="screen" pkg="gazebo_ros" type="spawn_model" args="-sdf -param model_description -model $(arg vehicle)_$(arg ID_in_group) -x $(arg x) -y $(arg y) -z $(arg z) -R $(arg R) -P $(arg P) -Y $(arg Y)"/>
</launch>
```

### Mine

```xml
<?xml version="1.0"?>
<launch>
    <!-- Posix SITL environment launch script -->
    <!-- launches PX4 SITL, Gazebo environment, and spawns vehicle -->
    <!-- vehicle pose -->
    <arg name="x" default="0"/>
    <arg name="y" default="0"/>
    <arg name="z" default="0"/>
    <arg name="R" default="0"/>
    <arg name="P" default="0"/>
    <arg name="Y" default="0"/>
    <!-- vehicle model and world -->
    <arg name="est" default="ekf2"/>
    <arg name="vehicle" default="iris"/>
    <arg name="world" default="$(find mavlink_sitl_gazebo)/worlds/empty.world"/>
    <arg name="sdf" default="$(find mavlink_sitl_gazebo)/models/$(arg vehicle)/$(arg vehicle).sdf"/>
    <env name="PX4_SIM_MODEL" value="$(arg vehicle)" />
    <env name="PX4_ESTIMATOR" value="$(arg est)" />

    <!-- gazebo configs -->
    <arg name="gui" default="true"/>
    <arg name="debug" default="false"/>
    <arg name="verbose" default="false"/>
    <arg name="paused" default="false"/>
    <arg name="respawn_gazebo" default="false"/>
    <!-- PX4 configs -->
    <arg name="interactive" default="true"/>
    <!-- PX4 SITL -->
    <arg unless="$(arg interactive)" name="px4_command_arg1" value="-d"/>
    <arg     if="$(arg interactive)" name="px4_command_arg1" value=""/>
    <node name="sitl" pkg="px4" type="px4" output="screen"
        args="$(find px4)/ROMFS/px4fmu_common -s etc/init.d-posix/rcS $(arg px4_command_arg1)" required="true"/>

    <!-- Gazebo sim -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="gui" value="$(arg gui)"/>
	<!-- change the world to the one I created -->
        <arg name="world_name" value="/home/hazyparker/project/my-research/3_Onboard_SLAM/gazebo_model/world/world_diy2.world"/>
        <arg name="debug" value="$(arg debug)"/>
        <arg name="verbose" value="$(arg verbose)"/>
        <arg name="paused" value="$(arg paused)"/>
        <arg name="respawn_gazebo" value="$(arg respawn_gazebo)"/>
    </include>
    <!-- gazebo model -->
    <node name="$(anon vehicle_spawn)" pkg="gazebo_ros" type="spawn_model" output="screen" args="-sdf -file $(arg sdf) -model $(arg vehicle) -x $(arg x) -y $(arg y) -z $(arg z) -R $(arg R) -P $(arg P) -Y $(arg Y)"/>
</launch>
```

 <arg name="vehicle" default="plane"/>

## Result

通过对比`px4`自带的`single_vehicle_spawn.launch`和`xtdrone`的该文件，发现差异在

```xml
<!-- vehicle model and config -->
    <arg name="est" default="ekf2"/>
    <arg name="vehicle" default="iris"/>
    <arg name="ID" default="1"/>
    <env name="PX4_SIM_MODEL" value="$(arg vehicle)" />
    <env name="PX4_ESTIMATOR" value="$(arg est)" />
    <arg name="mavlink_udp_port" default="14560"/>
    <arg name="mavlink_tcp_port" default="4560"/>
    <arg name="gst_udp_port" default="5600"/>
    <arg name="video_uri" default="5600"/>
    <arg name="mavlink_cam_udp_port" default="14530"/>
    <arg name="mavlink_id" value="$(eval 1 + arg('ID'))" />
```

```xml
    <!-- vehcile model and config -->
    <arg name="sdf" default="plane"/>
    <arg name="est" default="ekf2"/>
    <arg name="vehicle" default="plane"/>
    <arg name="ID" default="0"/>
    <arg name="ID_in_group" default="0"/>
    <env name="PX4_SIM_MODEL" value="$(arg vehicle)" />
    <env name="PX4_ESTIMATOR" value="$(arg est)" />
    <arg name="mavlink_udp_port" default="14560"/>
    <arg name="mavlink_tcp_port" default="4560"/>
```

这感觉问题也不大，关注`gst_udp_port`和`video_uri`都是什么参数。