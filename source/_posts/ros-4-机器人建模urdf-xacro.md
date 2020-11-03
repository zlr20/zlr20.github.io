---
title: ros(4)机器人建模urdf/xacro
date: 2020-11-02 15:38:40
tags:
- ROS
- Robotics
categories:
- ROS
---

本文总结了如何使用urdf和xacro对机器人进行建模。ROS中使用urdf文件描述一个机器人，ROS解析了该描述文件后，机器人才可以被rviz可视化、gazebo仿真或moveit运动规划等等。xacro则提供了一种更高层的编写方式，便于用户使用，但是实际使用时，均会先调用`rosrun xacro xacro.py xxx.urdf.xacro > xxx.urdf`先解析为urdf格式，再使用。

<!--More-->

## 1. urdf

### 1.1 基本结构

urdf全称unified robot description format，是一种特殊的xml文件格式。

一个机器人总是可以由树状的link描述，用joint连接父连杆与子连杆，这样就可以从基底逐级延伸到末端执行器。

<img src='http://i2.tiimg.com/728885/f80f318e2576e1c4.png' width=350>

因此urdf文件从最高层次有如下形式：

```xml
<robot name="my_test_robot">
	<link name="link1">
    	<!--xxx-->
    </link>
    <joint name="joint1" type="xxx">
        <!--xxx-->
    </joint>
    <link name="link2">
    	<!--xxx-->
    </link>
	...
</robot>
```

更精细地：



### 1.2 link

对于一个link而言，它拥有`visual`(用于外观可视化)、`collision`(用于碰撞检测)、`inertial`(用于动力学计算)三个标签。

```xml
<link name="base_link">
    <visual>
        <origin xyz=" 0 0 0" rpy="0 0 0" />
        <geometry>
            <cylinder length="0.16" radius="0.20"/>
        </geometry>
        <color rgba="1 0.4 0 1"/>
    </visual>
</link>
```

只有visual标签是必需的，且需要指定`geometry`(几何形状)、`origin`(几何中心坐标系相对于joint坐标系的偏移位姿)以及`color`。

collision标签也需要指定`geometry`和`origin`，一般来说，几何形状可以是圆柱、立方体等基本形状，也可以是solidworks等外部导入的模型文件，出于计算性能考虑，我们可以在visual中使用精确的模型，在collision中近似用简单几何体替代。

inertial标签包含动力学解算要用到的参数，包括`mass`(质量)以及`inertia`(惯性矩阵)。

### 1.3 joint

对于一个joint而言，首先需要指定它的类型：

| 关节类型   | 描述                             |
| ---------- | -------------------------------- |
| continuous | 旋转关节，可以围绕单轴无限旋转   |
| revolute   | 旋转关节，但是有旋转的角度限制   |
| prismatic  | 滑动关节，沿某一轴线，有位置极限 |
| fixed      | 固定关节，不允许运动             |
| floating   | 浮动关节（不常用）               |
| planar     | 平面关节（不常用）               |

joint拥有如下标签：

- parent：父连杆
- child：子连杆
- origin：相对于父连杆坐标系的位姿偏移
- axis：旋转轴
- limit：极限设置

```xml
<joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel_link"/>
    <origin xyz="0 0.19 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
</joint>
```

### 1.4 可视化

完整urdf文件：https://github.com/huchunxu/ros_basic_tutorials/blob/master/handwriting_urdf/mbot_description/urdf/mbot.urdf

```bash
roslaunch urdf_tutorial display.launch model:=path/to/your/xxx.urdf gui:=true
```

<img src='http://i2.tiimg.com/728885/87c2f66f5fac5aa3.png' width=550>



## 2. xacro

### 2.1 xacro与urdf的关系

xacro文件和urdf实质上是等价的，只不过xacro格式提供了一些更高级的方式来组织编辑机器人描述。借用在官方教程中一句话来形容xacro的优势: “Fortunately, you can use the [xacro](http://wiki.ros.org/xacro) package to make your life simpler”。

xacro主要提供了三种方式来使得整个描述文件变得简单：

- 像高级语言一样定义变量/属性，如指定PI，轮子半径，最大速度等，这样修改的时候全局生效，而不需要深入每一个link或joint去修改数值。
- 定义宏函数，相似的操作可以通过传入不同参数完成，方便重用。如定义四个对称的轮子。
- include其他xacro文件，进一步提高模块化和重用性。

### 2.2 xacro:property

用法：`<xacro:property name="MASS" value="2.0"/>`，在后面需要的时候以`${MASS}`调用。注意`${...}`中是可以加入数学运算的。

```xml
<!-- PROPERTY LIST -->
<xacro:property name="M_PI" value="3.1415926"/>
<xacro:property name="base_mass"   value="1" /> 
<xacro:property name="base_radius" value="0.20"/>
<xacro:property name="base_length" value="0.16"/>
....
....
<link name="base_link">
    <visual>
        <origin xyz=" 0 0 0" rpy="0 0 0" />
        <geometry>
            <cylinder length="${base_length}" radius="${base_radius}"/>
        </geometry>
        <material name="yellow" />
    </visual>
    <collision>
        <origin xyz=" 0 0 0" rpy="0 0 0" />
        <geometry>
            <cylinder length="${base_length}" radius="${base_radius}"/>
        </geometry>
    </collision>   
    <cylinder_inertial_matrix  m="${base_mass}" r="${base_radius}" h="${base_length}" />
</link>
```

### 2.2 xacro:macro

用法：`<xacro:macro name="fn_name" params="a b c">...</xacro:macro>`定义宏函数，注意指定函数名和参数。在后面需要的时候以`<fn_name a="1" b="2" c="3"/>`调用。有的文件中会以`<xacro:fn_name a="1" b="2" c="3"/>`调用，不要被迷惑了，这不是在定义宏，而是在使用宏。

```xml
<!-- Macro for robot wheel -->
<xacro:macro name="wheel" params="prefix reflect">
    <joint name="${prefix}_wheel_joint" type="continuous">
        <origin xyz="0 ${reflect*wheel_joint_y} ${-wheel_joint_z}" rpy="0 0 0"/>
        <parent link="base_link"/>
        <child link="${prefix}_wheel_link"/>
        <axis xyz="0 1 0"/>
    </joint>
    <link name="${prefix}_wheel_link">
        <visual>
            <origin xyz="0 0 0" rpy="${M_PI/2} 0 0" />
            <geometry>
                <cylinder radius="${wheel_radius}" length = "${wheel_length}"/>
            </geometry>
            <material name="gray" />
        </visual>
    </link>
</xacro:macro>
...
...
<wheel prefix="left"  reflect="1"/>
<wheel prefix="right" reflect="-1"/>
```

通过调用宏函数，我们不用重复造轮子了！

注意params中的prefix和reflect相当于局部变量，根据轮子的不同而指定，其他调用如`${wheel_joint_y}`是之前使用xacro:property指定的全局变量。

### 2.4 xacro:include

很多模型都是已宏的形式进行定义, 并以最小组成分成很多个文件. 而最终的机器人描述就变得非常简单了。 下面摘录一个ur5的描述文件. 从中可以看出来xacro的强大优势：

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="ur5" >

  <!-- common stuff -->
  <xacro:include filename="$(find ur_description)/urdf/ur5/common.gazebo.xacro" />

  <!-- ur5 -->
  <xacro:include filename="$(find ur_description)/urdf/ur5/ur5.urdf.xacro" />

  <!-- arm -->
  <xacro:ur5_robot prefix="" joint_limited="false"/>

  <link name="world" />

  <joint name="world_joint" type="fixed">
    <parent link="world" />
    <child link = "base_link" />
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
  </joint>

</robot>
```

从上述内容中可以看到, 首先是在ur_description包中找到另外几个xacro文件, 将其包含进来。当然应该注意到, include类似于C语言中的include, 先将该文件扩展到包含的位置. 但包含进来的文件很有可能只是一个参数宏的定义， 并没有被调用。所以, 示例中调用了一个宏`<xacro:ur5_robot prefix="" joint_limited="false"/>`产生一个ur5机器人。

### 2.5 可视化

```bash
roslaunch urdf_tutorial xacrodisplay.launch model:=path/to/your/xxx.urdf.xacro gui:=true
```



## 3. 使用案例

### 3.1 xx_description功能包

在一个实际机器人项目中，整个项目一般是一个元功能包，对于机器人的描述会建立并放置在一个`xxx_description`的功能包，比如`ur_description`就是ur机器人项目下的一个功能包。

以下面mbot_description功能包为例，完整工程参考：

https://github.com/huchunxu/ros_basic_tutorials/tree/master/handwriting_urdf/mbot_description 

建立时，要指明依赖ROS自带的urdf和xacro功能包，分别用于解析urdf和xacro文件。

```bash
catkin_create_pkg mbot_description urdf xacro
```

一般文件组织形式如下：

<img src='http://i2.tiimg.com/728885/ca08832987724fee.png'>

- urdf文件夹中放.urdf或.xacro文件
- meshes文件夹中放外部渲染模型
- launch文件中一般是在rviz中展示机器人模型的启动文件
- config文件中保存rviz配置

### 3.2 launch文件

xx_description下的launch文件一般是只在rviz中做可视化的（gazebo仿真一般会建新的功能包），用于简单可视化验证。

```xml
<launch>
	<!-- 设置机器人模型路径参数 -->
	<param name="robot_description" textfile="$(find mbot_description)/urdf/mbot.urdf" />

	<!-- 运行joint_state_publisher节点，发布机器人的关节状态  -->
	<node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" />
	
	<!-- 运行robot_state_publisher节点，发布tf  -->
	<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
	
	<!-- 运行rviz可视化界面 -->
	<node name="rviz" pkg="rviz" type="rviz" args="-d $(find mbot_description)/config/mbot_urdf.rviz" required="true" />
</launch>
```

如果是xacro定义的，要先使用xacro包解析：

```xml
<launch>
	<arg name="model" default="$(find xacro)/xacro --inorder '$(find mbot_description)/urdf/mbot_gazebo.xacro'" />
	<arg name="gui" default="true" />

	<param name="robot_description" command="$(arg model)" />

    <!-- 设置GUI参数，显示关节控制插件 -->
	<param name="use_gui" value="$(arg gui)"/>

    <!-- 运行joint_state_publisher节点，发布机器人的关节状态  -->
	<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />

	<!-- 运行robot_state_publisher节点，发布tf  -->
	<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />

    <!-- 运行rviz可视化界面 -->
	<node name="rviz" pkg="rviz" type="rviz" args="-d $(find mbot_description)/config/mbot.rviz" required="true" />

</launch>
```