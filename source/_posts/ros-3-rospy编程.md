---
title: ros(3)rospy编程
date: 2020-10-30 14:06:40
tags:
- ROS
- Robotics
categories:
- ROS
---

客户端库（Client Libarary）不仅仅指的是C++、Python语言的接口，其实是各种语言的接口统称。roscpp是最常用的接口，但是c++开发周期相对较长。本文我们来学习ROS的另一个接口rospy，也即是Python语言的接口。

<!--More-->

## 1. rospy vs roscpp

rospy是Python版本的ROS客户端库，提供了Python编程需要的接口，你可以认为rospy就是一个Python的模块(Module)。这个模块位于`/opt/ros/kineetic/lib/python2.7/dist-packages/rospy`之中。

rospy包含的功能与roscpp相似，都有关于node、topic、service、param、time相关的操作。但同时rospy和roscpp也有一些区别：

1. rospy没有一个NodeHandle，像创建publisher、subscriber等操作都被直接封装成了rospy中的函数或类，调用起来简单直观。
2. rospy一些接口的命名和roscpp不一致，有些地方需要开发者注意，避免调用错误。

相比于C++的开发，用Python来写ROS程序开发效率大大提高，诸如显示、类型转换等细节不再需要我们注意，节省时间。但Python的执行效率较低，同样一个功能用Python运行的耗时会高于C++。因此我们开发SLAM、路径规划、机器视觉等方面的算法时，往往优先选择C++。但ROS中绝大多数基本指令，例如`rostopic`,`roslaunch`都是用python开发的，简单轻巧。



## 2. 代码组织形式

对于一些小体量的ROS程序，一般就是一个Python文件，放在script/路径下，非常简单。	

```
pkg_name
|- script/
|---------|- xxx.py
```

当程序的功能比较复杂，放在一个脚本里搞不定时，就需要把一些功能放到Python Module里，以便其他的脚本来调用。ROS建议我们按照以下规范来建立一个Python的模块：

```
pkg_name
|- src/
|-------|-pkg_name/
|-------|-------------|- _init_.py
|-------|-------------|- modulefiles.py
|- scripts/
|-------|- xxx.py
|- setup.py
```

在src下建立一个与你的package同名的路径，其中存放`_init_.py`以及你的模块文件。这样就建立好了ROS规范的Python模块，你可以在你的脚本中调用。



## 3. Topic通信的实现

### 3.1 创建功能包

```bash
catkin_create_pkg topic_demo rospy std_msgs
```

### 3.2 定义.msg文件

在topic_demo/msg文件夹下创建一个自定义的gps类型的消息，一个节点发布模拟的gps信息，另一个接收和计算距离原点的距离。

`gps.msg`定义如下：

```
string state   #工作状态
float32 x      #x坐标
float32 y      #y坐标
```

我们需要修改`CMakeLists.txt`文件：

```cmake
find_package(catkin REQUIRED COMPONENTS
rospy
std_msgs
message_generation   #需要添加的地方
)

add_message_files(FILES gps.msg)  
#catkin在cmake之上新增的命令，指定从哪个消息文件生成

generate_messages(DEPENDENCIES std_msgs) 
#catkin新增的命令，用于生成消息
#DEPENDENCIES后面指定生成msg需要依赖其他什么消息，由于gps.msg用到了flaot32这种ROS标准消息，因此需要再把std_msgs作为依赖
```

`package.xml`中需要的改动：

```xml
<depend>message_generation</depend>
```

完成了以上所有工作，就可以回到工作空间，然后`catkin_make`了。

这里需要强调一点的就是，对创建的msg进行`catkin_make`会在`~/catkin_ws/devel/lib/python2.7/dist-packages/topic_demo`下生成msg模块（module）。 有了这个模块，我们就可以在python程序中`from topic_demo.msg import gps`,从而进行gps类型消息的读写。

### 3.3 话题发布节点

topic_demo/scripts/pytalker.py：

```python
#!/usr/bin/env python
#coding=utf-8
import rospy
#导入自定义的数据类型
from topic_demo.msg import gps

def talker():
    #Publisher函数第一个参数是话题名称，第二个参数数据类型，最后一个是缓冲区的大小
    #queue_size: None（不建议）  #这将设置为阻塞式同步收发模式！
    #queue_size: 0（不建议）#这将设置为无限缓冲区模式，很危险！
    #queue_size: 10 or more  #一般情况下，设为10 。queue_size太大了会导致数据延迟不同步。
    pub = rospy.Publisher('gps_info', gps , queue_size=10)
    rospy.init_node('pytalker', anonymous=True)
    #更新频率是1hz
    rate = rospy.Rate(1) 
    x=1.0
    y=2.0
    state='working'
    while not rospy.is_shutdown():
        rospy.loginfo('Talker: GPS: x=%f ,y= %f',x,y)
        pub.publish(gps(state,x,y))
        x=1.03*x
        y=1.01*y
        rate.sleep()

if __name__ == '__main__':
    talker()
```

以上代码与C++的区别体现在这几个方面：

1. rospy创建和初始化一个node，不再需要用NodeHandle。rospy中没有设计NodeHandle这个句柄，我们创建topic、service等等操作都直接用rospy里对应的方法就行。
2. rospy中节点的初始化并一定得放在程序的开头，在Publisher建立后再初始化也没问题。
3. 消息的创建更加简单，比如gps类型的消息可以直接用类似于构造函数的方式`gps(state,x,y)`来创建。
4. 日志的输出方式不同，C++中是`ROS_INFO()`，而Python中是`rospy.loginfo()`
5. 判断节点是否关闭的函数不同，C++用的是`ros::ok()`而Python中的接口是`rospy.is_shutdown()`

通过以上的区别可以看出，roscpp和rospy的接口并不一致，在名称上要尽量避免混用。在实现原理上，两套客户端库也有各自的实现，并没有基于一个统一的核心库来开发。这也是ROS在设计上不足的地方。

### 3.4 话题订阅节点

topic_demo/scripts/pylistener.py：

```cpp
#!/usr/bin/env python
#coding=utf-8
import rospy
import math
#导入mgs
from topic_demo.msg import gps

#回调函数输入的应该是msg
def callback(gps):
    distance = math.sqrt(math.pow(gps.x, 2)+math.pow(gps.y, 2)) 
    rospy.loginfo('Listener: GPS: distance=%f, state=%s', distance, gps.state)

if __name__ == '__main__':
    rospy.init_node('pylistener', anonymous=True)
    #Subscriber函数第一个参数是topic的名称，第二个参数是接受的数据类型 第三个参数是回调函数的名称
    rospy.Subscriber('gps_info', gps, callback)
    rospy.spin()
```

在订阅节点的代码里，rospy与roscpp有一个不同的地方：rospy里没有`spinOnce()`，只有`spin()`。

建立完talker和listener之后，经过`catkin_make`，就完成了python版的topic通信模型。

