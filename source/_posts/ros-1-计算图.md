---
title: ros(1)计算图
date: 2020-10-28 20:47:31
tags:
- ROS
- Robotics
categories:
- ROS
---

ROS的通信架构（计算图）是ROS的灵魂，也是整个ROS正常运行的关键所在。ROS通信架构包括各种数据的处理，进程的运行，消息的传递等等。本文主要介绍了通信架构的基础通信方式和相关概念。

<!--More-->

## 1. Node与Master

### 1.1 基本概念

**在ROS的计算图，最小的进程单元就是节点（node）。**从程序角度来说，node就是一个可执行文件（通常为C++编译生成的可执行文件、Python脚本）被执行，加载到了内存之中；从功能角度来说，通常一个node负责者机器人的某一个单独的功能。由于机器人的功能模块非常复杂，我们往往不会把所有功能都集中到一个node上，而会采用分布式的方式：如有一个node来控制底盘轮子的运动，有一个node驱动摄像头获取图像，有一个node驱动激光雷达，有一个node根据传感器信息进行路径规划……这样做可以降低系统发生崩溃的可能性。

我们使用过的小海龟的运动程序和键盘控制程序，在运行的时候便是两个独立的node，键盘命令通过ROS通信方式传递给小海龟仿真器，从而实现运动控制。我们可以把键盘控制替换为其他控制方式，而小海龟运动程序、机器人仿真程序则不用变化。这样就是一种模块化分工的思想。

**节点管理器（master）在整个计算图里相当于管理中心，管理着各个node。**由于机器人的元器件很多，功能庞大，因此实际运行时往往会运行众多的node，这就要利用ROS提供给我们的节点管理器master, 合理的进行调配、管理这些node。

管理主要体现在如下方面：

- node首先在master处进行注册，之后master会将该node纳入整个ROS计算图中。
- node之间的通信也是先由master进行“牵线”，才能两两的进行点对点通信。

<img src='http://i1.fuimg.com/728885/91e10aa96ee2feea.png' width=500>

### 1.2 启动Master和Node

当ROS计算图启动时，第一步首先启动master，再由节点管理器处理依次启动node，不然无法正常注册和管理。

```bash
roscore
```

注：伴随ROS Master启动的还有`rosout`和`parameter server`。其中`rosout`是负责日志输出的一个节点，其作用是告知用户当前系统的状态，包括输出系统的error、warning等等，并且将log记录于日志文件中，`parameter server`即是参数服务器，它并不是一个node，而是存储参数配置的一个服务器，后文我们会单独介绍。

master之后，节点管理器就开始按照系统的安排协调进行启动具体的节点。具体启动node的语句是：

```bash
rosrun pkg_name node_name
```

### 1.3 launch文件

通常一个机器人系统运行操作时要开启很多个node。当然，我们并不需要每个节点依次进行rosrun，ROS为我们提供了一个命令能一次性启动master和多个node。该命令是：

```
roslaunch pkg_name file_name.launch
```

roslaunch命令首先会自动进行检测系统的roscore有没有运行，也即是确认节点管理器是否在运行状态中，如果master没有启动，那么roslaunch就会首先启动master，然后再按照launch的规则执行。**launch文件里已经配置好了启动的规则。** 所以roslaunch就像是一个启动工具，能够一次性把多个节点按照我们预先的配置启动起来，减少我们在终端中一条条输入指令的麻烦。

launch文件遵循xml格式规范：

```xml
<launch>    <!--根标签-->
    <node>    <!--需要启动的node及其参数-->
    <include>    <!--包含其他launch-->
    <machine>    <!--指定运行的机器-->
    <env-loader>    <!--设置环境变量-->
    <param>    <!--定义参数到参数服务器-->
    <rosparam>    <!--启动yaml文件参数到参数服务器-->
    <arg>    <!--定义变量-->
    <remap>    <!--设定参数映射-->
    <group>    <!--设定命名空间-->
</launch>    <!--根标签-->
```

更具体的例子后面一定会遇到，这里先挖个坑。



## 2. Topic通信机制

### 2.1 基本概念

Topic是一种点对点的单向通信方式，对于实时性、周期性的消息，使用Topic来传输是最佳的选择。**Topic机制有点类似总线机制，发布者接入总线开始发，接收者接入总线开始收，他们之间是双盲的，只是之前由Master引入“同一根总线”(Topic)。**Topic机制是异步的，发布者只负责发布，不关心有没有人接受，也不关心接受的消息是否准确；同理接收者也是。因此他们之间不会存在任何协同工作。

<img src='http://i1.fuimg.com/728885/9d521dec25191594.jpg' width=500>

我们以摄像头画面的发布、处理、显示为例讲讲topic通信的流程。当node1（机器人上的摄像头拍摄程序）运行启动之后，它作为一个Publisher就开始发布topic，叫做`/camera_rgb`，即采集到的彩色图像。同时，node2（图像处理程序）订阅了`/camera_rgb`这个topic，经过节点管理器的介绍，它就能建立和摄像头节点（node1）的连接。

ROS是一种分布式的架构，一个topic可以被多个节点同时发布，也可以同时被多个节点接收。比如在这个场景中用户可以再加入一个图像显示的节点，我们在想看看摄像头节点的画面，则可以用自己的笔记本连接到机器人上的节点管理器，然后在自己的电脑上启动图像显示节点。

总结三点：

1. topic通信方式是异步的，发送时调用publish()方法，发送完成立即返回，不用等待反馈。
2. subscriber通过回调函数的方式来处理消息。所谓回调就是提前定义好了一个处理函数（写在代码中），当有消息来就会触发这个处理函数，函数会对消息进行处理。
3. topic可以同时有多个subscribers，也可以同时有多个publishers。ROS中这样的例子有：/rosout、/tf等等。

### 2.2 .msg文件

Topic有很严格的格式要求，由`.msg`文件定义。跑在某一条Topic“总线”上的每一条消息都是`.msg`所定义类型的实例。

我们看一个具体的`.msg`文件，例如`sensor_msg/image`,它的结构如下：

```
std_msg/Header header
    uint32    seq
    time    stamp
    string    frame_id
uint32    height
uint32    width
uint8[]    data
```

观察上面`.msg`文件的定义，很类似嵌套定义的结构体，通过具体的定义图像的宽度，高度等等来规范图像的格式。



## 3. Service通信机制

### 3.1 基本概念

有些时候总线型的通信满足不了任务要求，比如当一些节点只是临时而非周期性的需要某些数据，如果用topic通信方式时就会消耗大量不必要的系统资源，造成系统的低效率高功耗。这种情况下，就需要有另外一种**请求-查询式的通信模型**。

Service通信是双向的，包括两部分，一部分是请求方（Clinet），另一部分是应答方/服务提供方（Server）。Client就会发送一个request，要等待Server处理，反馈回一个reply，这样通过类似“请求-应答”的机制完成整个服务通信。

<img src='http://i1.fuimg.com/728885/3cb16183a8a933f6.png' width=500>

Service是同步通信方式，所谓同步就是说，此时Node A发布请求后会在原地等待reply，直到Node B处理完了请求并且完成了reply，Node A才会继续执行。Node A等待过程中，是处于阻塞状态的成通信。这样的通信模型没有频繁的消息传递，没有冲突与高系统资源的占用，只有接受请求才执行服务，简单而且高效。

### 3.2 .srv文件

类似`.msg`文件，`.srv`文件是用来描述服务类型的，它声明了一个服务，包括请求request和响应reply两部分。以`DetectHUman.srv`文件为例，该服务例子取自OpenNI的人体检测ROS软件包。它是用来查询当前深度摄像头中的人体姿态和关节数的。`.srv`文件格式很固定，第一行是请求的格式，中间用**---**隔开，第三行是应答的格式。

```
bool start_detect
---
my_pkg/HumanPose[] pose_data
```

## 4. Actionlib通信机制

### 4.1 基本概念

Actionlib是ROS中一个很重要的库，类似service通信机制，actionlib也是一种请求响应机制的通信方式，actionlib主要弥补了service通信的一个不足，就是当机器人执行一个长时间的任务时，假如利用service通信方式，那么client会很长时间接受不到反馈的reply，致使通信受阻。Actionlib可以比较适合实现长时间的通信过程，可以随时被查看过程进度，也可以终止请求，这样的一个特性，使得它在一些特别的机制中拥有很高的效率。

<img src='http://i2.tiimg.com/728885/07c3a75df31db846.png'>

可以看到,客户端会向服务器发送目标指令和取消动作指令,而服务器则可以给客户端发送实时的状态信息,结果信息,反馈信息等等,从而完成了service没法做到的部分。

### 4.2 .action文件

利用动作库进行请求响应，动作的内容格式应包含三个部分，目标、反馈、结果。机器人执行一个动作，应该有明确的移动目标信息，包括一些参数的设定，方向、角度、速度等等。从而使机器人完成动作任务。在动作进行的过程中，应该有实时的状态信息反馈给服务器的实施者，告诉实施者动作完成的状态，可以使实施者作出准确的判断去修正命令。当运动完成时，动作服务器把本次运动的结果数据发送给客户端，使客户端得到本次动作的全部信息，例如可能包含机器人的运动时长，最终姿势等等。

Action规范文件的后缀名是.action，它的内容格式如下：

```
# Define the goal
uint32 dishwasher_id  # Specify which dishwasher we want to use
---
# Define the result
uint32 total_dishes_cleaned
---
# Define a feedback message
float32 percent_complete
```

## 5.参数服务器

参数服务器是节点存储参数的地方、用于配置参数，全局共享参数。参数服务器维护着一个数据字典，字典里存储着各种参数和配置。参数服务器在节点管理器中运行。参数服务器的维护有三种方式：

- 命令行维护
- launch文件内读写
- node源码

### 5.1 命令行维护

使用命令行来维护参数服务器，主要使用`rosparam`语句来进行操作的各种命令。主要是:

```bash
rosparam set param_key param_value # 设置参数
rosparam get param_key # 显示参数
rosparam load file_name # 加载文件中的参数
rosparam dump file_name # 保存参数到文件
```

load和dump文件需要遵守YAML格式，YAML每一行都是`key : value`的格式。

### 5.2 launch文件读写

launch文件中与参数服务器相关的标签只有两个，一个是`<param>`，另一个是`<rosparam>`。这两个标签功能比较相近，但`<param>`一般只设置一个参数，`<rosparam>`的典型用法是先指定一个YAML文件，然后施加command,其效果等于`rosparam load file_name` 。

### 5.3 node源码

还可以在编写代码时候设置，后面具体写代码再看。



