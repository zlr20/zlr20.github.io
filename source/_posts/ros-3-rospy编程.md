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



## 4. Service通信的实现

### 4.1 创建功能包

```bash
catkin_create_pkg service_demo rospy std_msgs
```

### 4.2 定义.srv文件

在service_demo/srv目录下一个名为`Greeting.srv`的服务文件，内容如下：

```
string name 
int32 age
# 短横线上边部分是服务请求的数据
--- 
# 短横线下面是服务回传的内容
string feedback
```

然后修改`CMakeLists.txt`文件。ROS的catkin编译系统会将你自定义的msg、srv（甚至还有action）文件自动编译构建，生成对应的Python语言下可用的库或模块。许多初学者错误地以为，只要建立了一个.msg或.srv文件，就可以直接在程序中使用，这是不对的，必须在`CMakeLists.txt`中添加关于消息创建、指定消息/服务文件那几个宏命令并完成编译才能生成可用的消息或服务内容。

事实上，对于服务，会生成三个类：Greeting, GreetingRequese以及GreetingResponse。

### 4.3 服务提供节点

service_demo/scripts/server_demo.py：

```python
#!/usr/bin/env python
#coding=utf-8
import rospy
from service_demo.srv import *

def handle_function(req):
    # 注意我们是如何调用request请求内容的，是将其认为是一个对象的属性，在我们定义
    # 的Service_demo类型的service中，request部分的内容包含两个变量，一个是字符串类型的name，另外一个是整数类型的age
    rospy.loginfo( 'Request from %s with age %d', req.name, req.age)
    # 返回一个Service_demo.Response实例化对象，其实就是返回一个response的对象，其包含的内容为我们在Service_demo.srv中定义的
    # response部分的内容，我们定义了一个string类型的变量feedback，因此，此处实例化时传入字符串即可
    return GreetingResponse("Hi %s. I' server!"%req.name)

# 如果单独运行此文件，则将上面定义的server_srv作为主函数运行
if __name__=="__main__":
    # 初始化节点，命名为 "greetings_server"
    rospy.init_node("greetings_server")
    # 定义service的server端，service名称为"greetings"， service类型为Greeting
    # 收到的request请求信息将作为参数传递给handle_function进行处理
    s = rospy.Service("greetings", Greeting, handle_function)
    rospy.loginfo("Ready to handle the request:")
    # 阻塞程序结束
    rospy.spin()
```

handle_function()传入的只有request，返回值是response，即：

```python
def handle_function(req):
    ...
    return GreetingResponse("Hi %s. I' server!"%req.name)
```

### 4.4 服务请求节点

service_demo/scripts/client_demo.py：

```python
#!/usr/bin/env python
# coding:utf-8
import rospy
from service_demo.srv import *

if __name__=="__main__":
    rospy.init_node('greetings_client')
    # 等待有可用的服务 "greetings"
    rospy.wait_for_service("greetings")
    try:
        # 定义service客户端，service名称为“greetings”，service类型为Greeting
        greetings_client = rospy.ServiceProxy("greetings",Greeting)
        # 向server端发送请求，发送的request内容为name和age,其值分别为"HAN", 20
        # 此处发送的request内容与srv文件中定义的request部分的属性是一致的
        resp = greetings_client.call("HAN",20)
        rospy.loginfo("Message From server:%s"%resp.feedback)
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed: %s"%e)
```

以上代码中`greetings_client.call("HAN",20)`等同于`greetings_client("HAN",20)`。



## 5. 参数服务器

rospy关于参数服务器的API包括了增删查改等用法： `rospy.get_param()`，`rospy.set_param()`，`rospy.has_param()`，`rospy.delete_param()`，`rospy.search_param()`，`rospy.get_param_names()`。

```python
#!/usr/bin/env python
# coding:utf-8
import rospy

def param_demo():
    rospy.init_node("param_demo")
    rate = rospy.Rate(1)
    while(not rospy.is_shutdown()):
        #get param
        parameter1 = rospy.get_param("/param1")
        parameter2 = rospy.get_param("/param2", default=222)
        rospy.loginfo('Get param1 = %d', parameter1)
        rospy.loginfo('Get param2 = %d', parameter2)

        #delete param
        rospy.delete_param('/param2')

        #set param
        rospy.set_param('/param2',2)

        #check param
        ifparam3 = rospy.has_param('/param3')
        if(ifparam3):
            rospy.loginfo('/param3 exists')
        else:
            rospy.loginfo('/param3 does not exist')

        #get all param names
        params = rospy.get_param_names()
        rospy.loginfo('param list: %s', params)

        rate.sleep()

if __name__=="__main__":
    param_demo()
```