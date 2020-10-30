---
title: ros(2)工作空间
date: 2020-10-29 16:10:01
tags:
- ROS
- Robotics
categories:
- ROS
---

本文介绍了ROS的工作空间，也就是ROS的文件系统结构。要学会建立一个ROS工程，首先要认识一个ROS工程，了解它们的组织架构，从根本上熟悉ROS项目的组织形式，了解各个文件的功能和作用，才能正确的进行开发和编程。

<!--More-->

## 1. Catkin编译系统

### 1.1 基本概念

对于源代码包，我们只有编译才能在系统上运行。而Linux下的编译器有gcc、g++，随着源文件的增加，直接用gcc/g++命令的方式显得效率低下，人们开始用Makefile来进行编译。然而随着工程体量的增大，Makefile也不能满足需求，于是便出现了Cmake工具。CMake是对make工具的生成器，是更高层的工具，它简化了编译构建过程，能够管理大型项目，具有良好的扩展性。对于ROS这样大体量的平台来说，就采用的是CMake，并且ROS对CMake进行了扩展，于是便有了Catkin编译系统，因此**Catkin是基于CMake的编译构建系统**。

### 1.2 工作空间结构

**Catkin工作空间是创建、修改、编译catkin软件包的目录**。catkin的工作空间，直观的形容就是一个仓库，里面装载着ROS的各种项目工程，便于系统组织管理调用。在可视化图形界面里是一个文件夹。我们自己写的ROS代码通常就放在工作空间中，catkin工作空间的结构如下：

<img src='http://i1.fuimg.com/728885/7f08cc33fc26c635.jpg' width=500>

这三个文件夹是catkin编译系统默认的。它们的具体作用如下：

- src/: ROS的catkin软件包（源代码包）
- build/: catkin（CMake）的缓存信息和中间文件
- devel/: 生成的目标文件（包括头文件，动态链接库，静态链接库，可执行文件等）、环境变量

src文件中放着我们的功能包package，每个功能包实现一个特定的功能。

<img src='http://i1.fuimg.com/728885/4475ac307c3beedd.jpg' width=500>

**package是catkin工作空间的基本单元**，我们在ROS开发时，写好代码，然后catkin_make，系统就会完成所有编译构建的工作。Catkin编译的工作流程如下：

1. 首先在工作空间`catkin_ws/src/`下递归的查找其中每一个ROS的package。
2. package中会有`package.xml`和`CMakeLists.txt`文件，Catkin(CMake)编译系统依据`CMakeLists.txt`文件,从而生成`makefiles`(放在`catkin_ws/build/`)。
3. 然后`make`刚刚生成的`makefiles`等文件，编译链接生成可执行文件(放在`catkin_ws/devel`)。

也就是说，Catkin就是将`cmake`与`make`指令做了一个封装从而完成整个编译过程的工具。

需要注意的是，我们只有在工作空间根目录下才能执行`catkin_make`。

### 1.3 构建工作空间

```bash
mkdir -p ~/catkin_ws/src　　
cd ~/catkin_ws/
catkin_make #初始化工作空间
```



## 2. Package功能包

### 2.1 基本概念

ROS中的package的定义是catkin编译的基本单元，是集中完成一个任务的代码合集。一个package可以编译出来多个目标文件（ROS可执行程序、动态静态库、头文件等等）。

### 2.2 功能包结构

一个package下常见的文件、路径有：

```
  ├── CMakeLists.txt    #package的编译规则(必须)
  ├── package.xml       #package的描述信息(必须)
  ├── src/              #源代码文件
  ├── include/          #C++头文件
  ├── scripts/          #可执行脚本
  ├── msg/              #自定义消息
  ├── srv/              #自定义服务
  ├── models/           #3D模型文件，配合urdf用于仿真
  ├── urdf/             #urdf文件，机器人描述
  ├── launch/           #launch文件，启动配置
  ├── param/            #yaml文件，参数
```

其中定义package的是`CMakeLists.txt`和`package.xml`，这两个文件是package中必不可少的。catkin编译系统在编译前，首先就要解析这两个文件。这两个文件就定义了一个package。

- CMakeLists.txt: 定义package的包名、依赖、源文件、目标文件等编译规则，是package不可少的成分
- package.xml: 描述package的包名、版本号、作者、依赖等信息，是package不可少的成分
- src/: 存放ROS的源代码，包括C++的源码和(.cpp)以及Python的module(.py)
- include/: 存放C++源码对应的头文件
- scripts/: 存放可执行脚本，例如shell脚本(.sh)、Python脚本(.py)
- msg/: 存放自定义格式的消息(.msg)
- srv/: 存放自定义格式的服务(.srv)
- models/: 存放机器人或仿真场景的3D模型(.sda, .stl, .dae等)
- urdf/: 存放机器人的模型描述(.urdf或.xacro)
- launch/: 存放launch文件(.launch或.xml)

通常ROS文件组织都是按照以上的形式，这是约定俗成的命名习惯，建议遵守。以上路径中，只有`CMakeLists.txt`和`package.xml`是必须的，其余路径根据软件包是否需要来决定。

### 2.3 构建功能包

```bash
catkin_create_pkg pkg_name [dependencies ...]
```



## 3. CMakeList.txt

`CMakeLists.txt`是Cmake编译系统的规则文件，而Catkin编译系统基本沿用了CMake的编译风格，只是针对ROS工程添加了一些宏定义。所以在写法上，catkin的`CMakeLists.txt`与CMake的基本一致。

这个文件直接规定了这个package要依赖哪些package，要编译生成哪些目标，如何编译等等流程。所以`CMakeLists.txt`非常重要，它指定了由源码到目标文件的规则，catkin编译系统在工作时首先会找到每个package下的`CMakeLists.txt`，然后按照规则来编译构建。

`CMakeLists.txt`的基本语法都还是按照CMake，而Catkin在其中加入了少量的宏，总体的结构如下：

```cmake
cmake_minimum_required() #CMake的版本号 
project()                #项目名称 
find_package()           #找到编译需要的其他CMake/Catkin package
catkin_python_setup()    #catkin新加宏，打开catkin的Python Module的支持
add_message_files()      #catkin新加宏，添加自定义Message/Service/Action文件
add_service_files()
add_action_files()
generate_message()       #catkin新加宏，生成不同语言版本的msg/srv/action接口
catkin_package()         #catkin新加宏，生成当前package的cmake配置，供依赖本包的其他软件包调用
add_library()            #生成库
add_executable()         #生成可执行二进制文件
add_dependencies()       #定义目标文件依赖于其他目标文件，确保其他目标已被构建
target_link_libraries()  #链接
catkin_add_gtest()       #catkin新加宏，生成测试
install()                #安装至本机
```

如果你想了解CMake的语法，请阅读《CMake实践》：https://github.com/Akagi201/learning-cmake/blob/master/docs/cmake-practice.pdf 。但是如果你用rospy做开发，基本不会用到太多，最常用的就是`add_xxx_files()`和`generate_message()`生成自定义的`msg/srv/action`接口。还有就是`catkin_python_setup()`与setup.py对应，对于包含自定义Python模块和包的场景不能少。



## 4. package.xml

`package.xml`也是一个catkin的package必备文件，它是这个软件包的描述文件。`pacakge.xml`包含了package的名称、版本号、内容描述、维护人员、软件许可、编译构建工具、编译依赖、运行依赖等信息。实际上`rospack find`、`rosdep`等命令之所以能快速定位和分析出package的依赖项信息，就是直接读取了每一个package中的`package.xml`文件。它为用户提供了快速了解一个package的渠道。

`pacakge.xml`遵循xml标签文本的写法，包含的标签为：

```xml
<pacakge>               根标记文件  
<name>                  包名  
<version>               版本号  
<description>           内容描述  
<maintainer>            维护者 
<license>               软件许可证  
<buildtool_depend>      编译构建工具，通常为catkin    
<depend>                指定依赖项为编译、导出、运行需要的依赖，最常用
<build_depend>          编译依赖项  
<build_export_depend>   导出依赖项
<exec_depend>           运行依赖项
<test_depend>           测试用例依赖项  
<doc_depend>            文档依赖项
```

注：指定了`<depend>XXX</depend>`相当于同时指定了`<build_depend>`  `<exec_depend>`等，不需要分开写了。如果有什么项目是只有编译依赖或只有运行依赖的，才再`<build_depend>`  或`<exec_depend>`上指定。



## 5. Metapackage元功能包

这部分内容只有在超大型项目才用得到，这里只是出于知识的完整性加以简单介绍。Metapackage在之前的版本中叫Stack(功能包集)，指的是将多个功能接近、甚至相互依赖的软件包的放到一个集合中去。

moveit是ROS中机械臂运动规划相关的元功能包，项目地址https://github.com/ros-planning/moveit。

<img src='http://i1.fuimg.com/728885/3dddd644f39ca6fe.png' width=550>

打开moveit元功能包目录：

<img src='http://i1.fuimg.com/728885/5e3aca32dae15d7b.png' width=550>

没有源码，只有`CMakeList.txt`和`package.xml`。

`CMakeLists.txt`写法如下：

```cmake
cmake_minimum_required(VERSION 3.1.3)
project(moveit)
find_package(catkin REQUIRED)
catkin_metapackage()   #声明本软件包是一个metapacakge
```

`pacakge.xml`写法如下：

```xml
<?xml version="1.0"?>
<package format="2">
  <name>moveit</name>
  <version>1.1.1</version>
  <description>Meta package that contains all essential package of MoveIt. Until Summer 2016 MoveIt had been developed over multiple repositories, where developers' usability and maintenance effort was non-trivial. See <a href = "http://discourse.ros.org/t/migration-to-one-github-repo-for-moveit/266/34">the detailed discussion for the merge of several repositories</a>.</description>
  <maintainer email="dave@picknik.ai">Dave Coleman</maintainer>
  <maintainer email="mferguson@fetchrobotics.com">Michael Ferguson</maintainer>
  <maintainer email="me@v4hn.de">Michael Görner</maintainer>
  <maintainer email="rhaschke@techfak.uni-bielefeld.de">Robert Haschke</maintainer>
  <maintainer email="imcmahon01@gmail.com">Ian McMahon</maintainer>
  <maintainer email="130s@2000.jukuin.keio.ac.jp">Isaac I. Y. Saito</maintainer>

  <license>BSD</license>

  <url type="website">http://moveit.ros.org</url>
  <url type="repository">https://github.com/ros-planning/moveit</url>
  <url type="bugtracker">https://github.com/ros-planning/moveit/issues</url>

  <author email="isucan@google.com">Ioan Sucan</author>
  <author email="robot.moveit@gmail.com">Sachin Chitta</author>

  <buildtool_depend>catkin</buildtool_depend>
  <!--运行时依赖与其他实体功能包-->
  <exec_depend>moveit_commander</exec_depend>
  <exec_depend>moveit_core</exec_depend>
  <exec_depend>moveit_planners</exec_depend>
  <exec_depend>moveit_plugins</exec_depend>
  <exec_depend>moveit_ros</exec_depend>
  <exec_depend>moveit_setup_assistant</exec_depend>

  <export>
    <metapackage/>  <!--这里需要有export和metapacakge标签，注意这种固定写法-->
  </export>
</package>
```

个人感觉有点像Python写包时候的`__init__.py`，将所有模块并进了同一个命名空间。