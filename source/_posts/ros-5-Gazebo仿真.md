---
title: ros(5)Gazebo仿真
date: 2020-11-05 14:45:43
tags:
- ROS
- Robotics
categories:
- ROS
---

Gazebo是ROS默认的仿真平台，我们可以通过Gazebo仿真器进行机器人仿真实验。本文介绍了如何在Gazebo仿真器中对机器人进行仿真、控制、添加传感器以及获取传感器数据。

<!--More-->

## 1. 完善.xarco文件

### 1.1 颜色系统

Gazebo的颜色系统和rviz不一样，原先指定的`<color rgba= 0 0 0 0.95/>`（黑色）会失效，在Gazebo中统一都是显示默认的灰色。因此如需要指定颜色，要使用Gazebo自己的颜色系统：

```xml
<xacro:macro name="wheel" params="prefix reflect">
	...
    <gazebo reference="${prefix}_wheel_link">
        <material>Gazebo/Gray</material>
    </gazebo>
</xacro:macro>
```

### 1.2 transmission

如果要使用Gazbo仿真，一般要指定joint的传动方式：位置模式、速度模式、力矩模式。这对之后添加相应控制器很重要，或者说控制器是作用在所指定的传动装置上的。好比添了一个电机。

比如，我们指定小车的每个轮子都是速度控制模式的：

```xml
<xacro:macro name="wheel" params="prefix reflect">
	...
    <gazebo reference="${prefix}_wheel_link">
        <material>Gazebo/Gray</material>
    </gazebo>
    <!-- Transmission is important to link the joints and the controller -->
    <transmission name="${prefix}_wheel_joint_trans">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="${prefix}_wheel_joint" >
            <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
        </joint>
        <actuator name="${prefix}_wheel_joint_motor">
            <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
            <mechanicalReduction>1</mechanicalReduction>
        </actuator>
    </transmission>
</xacro:macro>
```

## 2. 添加控制器

### 2.1 ros_control

<img src='http://i2.tiimg.com/728885/db356ee861c61f29.png'>

[ros_control](http://wiki.ros.org/ros_control)是ROS为用户提供的应用与机器人之间的中间件，包含一系列控制器接口、传动装置接口、硬件接口、控制器工具箱等等，可以帮助机器人应用快速落地，提高开发效率。上图是ros_control的总体框架，可以看到正对不同类型的控制器（底盘、机械臂等），ros_control可以提供多种类型的控制器，但是这些控制器的接口各不相同，为了提高代码的复用率，ros_control还提供一个硬件的抽象层。硬件抽象层负责机器人硬件资源的管理，而controller从抽象层请求资源即可，并不直接接触硬件。

### 2.2 Gazbo中的控制器插件

可以在.xacro文件中引入Gazbo的ros_control插件，完成指定的控制任务。如下面添加一个差速控制器：

```xml
<!-- controller -->
<gazebo>
    <plugin name="differential_drive_controller" 
            filename="libgazebo_ros_diff_drive.so">
        <rosDebugLevel>Debug</rosDebugLevel>
        <publishWheelTF>true</publishWheelTF>
        <robotNamespace>/</robotNamespace>
        <publishTf>1</publishTf>
        <publishWheelJointState>true</publishWheelJointState>
        <alwaysOn>true</alwaysOn>
        <updateRate>100.0</updateRate>
        <legacyMode>true</legacyMode>
        <leftJoint>left_wheel_joint</leftJoint>
        <rightJoint>right_wheel_joint</rightJoint>
        <wheelSeparation>${wheel_joint_y*2}</wheelSeparation>
        <wheelDiameter>${2*wheel_radius}</wheelDiameter>
        <broadcastTF>1</broadcastTF>
        <wheelTorque>30</wheelTorque>
        <wheelAcceleration>1.8</wheelAcceleration>
        <commandTopic>cmd_vel</commandTopic>
        <odometryFrame>odom</odometryFrame> 
        <odometryTopic>odom</odometryTopic> 
        <robotBaseFrame>base_footprint</robotBaseFrame>
    </plugin>
</gazebo> 
```

### 2.3 如何控制

此时在Gazebo中，多了`/cmd_vel`话题，订阅者就是仿真器，因此我们只要发布该话题要求的消息（这里是`Twist`，包含三轴线速度和三轴角速度），就可以控制小车移动。

- 方式一：终端直接rostopic pub...
- 方式二：编写一个node，使用键盘控制。像小海龟那样。

<img src='http://i1.fuimg.com/728885/9e0ce2e5235dd441.png'>

键盘控制程序：

https://github.com/huchunxu/ros_basic_tutorials/blob/master/mbot_gazebo/mbot_teleop/scripts/mbot_teleop.py

## 3. 添加传感器

### 3.1 camera

Gazebo为我们建立了摄像头插件`libgazebo_ros_camera.so`，只需调用这个插件并指定相关参数就可以添加仿真的摄像头传感器。但是我们一般不直接在机器人本体上加，而是把单独的一个传感器作为一个模块，按需使用。

建立camera.xacro，只定义一个宏函数，要用的时候在机器人本体xacro文件include它，并指定一下连接的joint即可：

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="camera">

    <xacro:macro name="usb_camera" params="prefix:=camera">
        <!-- Create laser reference frame -->
        <link name="${prefix}_link">
            <inertial>
                <mass value="0.1" />
                <origin xyz="0 0 0" />
                <inertia ixx="0.01" ixy="0.0" ixz="0.0"
                         iyy="0.01" iyz="0.0"
                         izz="0.01" />
            </inertial>

            <visual>
                <origin xyz=" 0 0 0 " rpy="0 0 0" />
                <geometry>
                    <box size="0.01 0.04 0.04" />
                </geometry>
                <material name="black"/>
            </visual>

            <collision>
                <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
                <geometry>
                    <box size="0.01 0.04 0.04" />
                </geometry>
            </collision>
        </link>
        <gazebo reference="${prefix}_link">
            <material>Gazebo/Black</material>
        </gazebo>

        <gazebo reference="${prefix}_link">
            <sensor type="camera" name="camera_node">
                <update_rate>30.0</update_rate>
                <camera name="head">
                    <horizontal_fov>1.3962634</horizontal_fov>
                    <image>
                        <width>1280</width>
                        <height>720</height>
                        <format>R8G8B8</format>
                    </image>
                    <clip>
                        <near>0.02</near>
                        <far>300</far>
                    </clip>
                    <noise>
                        <type>gaussian</type>
                        <mean>0.0</mean>
                        <stddev>0.007</stddev>
                    </noise>
                </camera>
                <plugin name="gazebo_camera" filename="libgazebo_ros_camera.so">
                    <alwaysOn>true</alwaysOn>
                    <updateRate>0.0</updateRate>
                    <cameraName>/camera</cameraName>
                    <imageTopicName>image_raw</imageTopicName>
                    <cameraInfoTopicName>camera_info</cameraInfoTopicName>
                    <frameName>camera_link</frameName>
                    <hackBaseline>0.07</hackBaseline>
                    <distortionK1>0.0</distortionK1>
                    <distortionK2>0.0</distortionK2>
                    <distortionK3>0.0</distortionK3>
                    <distortionT1>0.0</distortionT1>
                    <distortionT2>0.0</distortionT2>
                </plugin>
            </sensor>
        </gazebo>

    </xacro:macro>
</robot>
```

在机器人顶层描述文件中将机器人与传感器拼合：

```xml
<?xml version="1.0"?>
<robot name="arm" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <xacro:include filename="$(find mbot_description)/urdf/mbot_base_gazebo.xacro" />
    <xacro:include filename="$(find mbot_description)/urdf/sensors/camera_gazebo.xacro" />

    <xacro:property name="camera_offset_x" value="0.17" />
    <xacro:property name="camera_offset_y" value="0" />
    <xacro:property name="camera_offset_z" value="0.10" />

    <!-- Camera -->
    <joint name="camera_joint" type="fixed">
        <origin xyz="${camera_offset_x} ${camera_offset_y} ${camera_offset_z}" rpy="0 0 0" />
        <parent link="base_link"/>
        <child link="camera_link"/>
    </joint>

    <xacro:usb_camera prefix="camera"/>

    <mbot_base_gazebo/>

</robot>
```

在仿真器中加载机器人模型和场景环境，查看rostpic list可知，出现了图像相关话题，使用`image_view`功能包订阅相关话题，便可以看到传感器回传的图像了。另外，使用键盘控制小车移动，可以看到摄像头读取的图像一直在变化。如图，显示的就是正对着的那个木柜子：

<img src='http://i1.fuimg.com/728885/66797de94587ebb0.png'>

### 3.2 kinect

原理一样，插件写法如下：

```xml
<gazebo reference="${prefix}_link">
    <sensor type="depth" name="${prefix}">
        <always_on>true</always_on>
        <update_rate>20.0</update_rate>
        <camera>
            <horizontal_fov>${60.0*M_PI/180.0}</horizontal_fov>
            <image>
                <format>R8G8B8</format>
                <width>640</width>
                <height>480</height>
            </image>
            <clip>
                <near>0.05</near>
                <far>8.0</far>
            </clip>
        </camera>
        <plugin name="kinect_${prefix}_controller" filename="libgazebo_ros_openni_kinect.so">
            <cameraName>${prefix}</cameraName>
            <alwaysOn>true</alwaysOn>
            <updateRate>10</updateRate>
            <imageTopicName>rgb/image_raw</imageTopicName>
            <depthImageTopicName>depth/image_raw</depthImageTopicName>
            <pointCloudTopicName>depth/points</pointCloudTopicName>
            <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
            <depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>
            <frameName>${prefix}_frame_optical</frameName>
            <baseline>0.1</baseline>
            <distortion_k1>0.0</distortion_k1>
            <distortion_k2>0.0</distortion_k2>
            <distortion_k3>0.0</distortion_k3>
            <distortion_t1>0.0</distortion_t1>
            <distortion_t2>0.0</distortion_t2>
            <pointCloudCutoff>0.4</pointCloudCutoff>
        </plugin>
    </sensor>
</gazebo>
```

在gazebo中仿真，控制小车移动：

<img src='http://i2.tiimg.com/728885/513959e655940a7c.png'>

在rviz中同时可视化kinect回传的点云和图像：

<img src='http://i2.tiimg.com/728885/0c971db3bba5d4fb.png'>

### 3.3 激光雷达

原理一样，插件写法如下：的

```xml
<gazebo reference="${prefix}_link">
    <sensor type="ray" name="rplidar">
        <pose>0 0 0 0 0 0</pose>
        <visualize>false</visualize>
        <update_rate>5.5</update_rate>
        <ray>
            <scan>
              <horizontal>
                <samples>360</samples>
                <resolution>1</resolution>
                <min_angle>-3</min_angle>
                <max_angle>3</max_angle>
              </horizontal>
            </scan>
            <range>
              <min>0.10</min>
              <max>6.0</max>
              <resolution>0.01</resolution>
            </range>
            <noise>
              <type>gaussian</type>
              <mean>0.0</mean>
              <stddev>0.01</stddev>
            </noise>
        </ray>
        <plugin name="gazebo_rplidar" filename="libgazebo_ros_laser.so">
            <topicName>/scan</topicName>
            <frameName>laser_link</frameName>
        </plugin>
    </sensor>
</gazebo>
```

在rviz中显示激光雷达扫描效果：

<img src='http://i2.tiimg.com/728885/d32488e5108cd26f.png'>



## 4. launch文件

```xml
<launch>
    <!-- 设置launch文件的参数 -->
    <arg name="world_name" value="$(find mbot_gazebo)/worlds/playground.world"/>
    <arg name="paused" default="false"/>
    <arg name="use_sim_time" default="true"/>
    <arg name="gui" default="true"/>
    <arg name="headless" default="false"/>
    <arg name="debug" default="false"/>

    <!-- 运行gazebo仿真环境 -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name" value="$(arg world_name)" />
        <arg name="debug" value="$(arg debug)" />
        <arg name="gui" value="$(arg gui)" />
        <arg name="paused" value="$(arg paused)"/>
        <arg name="use_sim_time" value="$(arg use_sim_time)"/>
        <arg name="headless" value="$(arg headless)"/>
    </include>

    <!-- 加载机器人模型描述参数 -->
    <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find mbot_description)/urdf/mbot_gazebo.xacro'" /> 

    <!-- 运行joint_state_publisher节点，发布机器人的关节状态  -->
    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" ></node> 

    <!-- 运行robot_state_publisher节点，发布tf  -->
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"  output="screen" >
        <param name="publish_frequency" type="double" value="50.0" />
    </node>

    <!-- 在gazebo中加载机器人模型-->
    <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
          args="-urdf -model mrobot -param robot_description"/> 

</launch>
```

设置launch文件的参数指定Gazebo的行为：

- paused：在暂停状态下启动Gazebo（默认为false）
- use_sim_time：告诉ROS节点要求获取ROS话题/clock发布的时间信息（默认为true）
- gui：启动Gazebo中的用户界面窗口（默认为true）
- headless：启动Gazebo状态日志记录（默认为false）
- debug：使用gdb以调试模式启动gzserver（默认为false）
- verbose：用--verbose运行gzserver和gzclient，并将错误和警告打印到终端（默认为false）

最重要的是world_name参数，它使用指定的`.world`文件（world文件是Gazebo保存场景的文件）替换了empty_world。

加载机器人模型的方法是使用了spawn_model的python程序来要求gazebo_ros节点向Gazebo中添加URDF．spawn_model程序存储在gazebo_ros包中。

常用的除了上面的，还有`-x -y -z`指定初始坐标。要看所有的spawn_model中的可变参数，运行：rosrun gazebo_ros spawn_model -h。