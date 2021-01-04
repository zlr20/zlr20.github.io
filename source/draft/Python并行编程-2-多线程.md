---
title: Python并行编程(2)多线程
date: 2020-12-10 14:20:10
tags:
- Python
categories:
- Python
---

# 1. 前言

目前，在软件应用中使用最广泛的并发编程范例是多线程。通常，一个应用有一个进程，分成多个独立的线程，并行运行、互相配合，执行不同类型的任务。虽然这种模式存在一些缺点，有很多潜在的问题，但是多线程的应用依然非常广泛。现在几乎所有的操作系统都支持多线程，几乎所有的编程语言都有相应的多线程机制，可以在应用中通过线程实现并发。所以，使用多线程编程来实现并发的并用是个不错的选择。

线程是独立的处理流程，可以和系统的其他线程并行或并发地执行。多线程可以共享数据和资源，利用所谓的共享内存空间。线程和进程的具体实现取决于你要运行的操作系统，但是总体来讲，我们可以说线程是包含在进程中的，同一进程的多个不同的线程可以共享相同的资源。相比而言，进程之间不会共享资源。每一个线程基本上包含3个元素：程序计数器，寄存器和栈。与同一进程的其他线程共享的资源基本上包括数据和系统资源。每一个线程也有自己的运行状态，可以和其他线程同步，这点和进程一样。线程的状态大体上可以分为ready,running,blocked。线程的典型应用是应用软件的并行化——为了充分利用现代的多核处理器，使每个核心可以运行单个线程。相比于进程，使用线程的优势主要是性能。相比之下，在进程之间切换上下文要比在统一进程的多线程之间切换上下文要重的多。多线程编程一般使用共享内容空间进行线程间的通讯。这就使管理内容空间成为多线程编程的重点和难点。

# 2. 使用Python的线程模块

Python通过标准库的 `threading` 模块来管理线程。这个模块提供了很多不错的特性，让线程变得无比简单。实际上，线程模块提供了几种同时运行的机制，实现起来非常简单。

`threading`模块的主要组件如下：

- 线程对象
- Lock对象
- RLock对象
- 信号对象
- 条件对象
- 事件对象

# 3. 创建一个线程

## 3.1 直接使用Thread对象

Python中一切皆对象，线程也不例外。

创建线程最简单的一个方法是，用一个目标函数实例化一个 `Thread()`类的对象，然后调用 `start()` 方法启动它。

```python
class threading.Thread(group=None,
                       target=None,
                       name=None,
                       args=(),
                       kwargs={})
```

参数解释：

- `group`: 一般设置为 `None` ，这是为以后的一些特性预留的
- `target`: 当线程启动的时候要执行的函数
- `name`: 线程的名字，默认会分配一个唯一名字 `Thread-N`
- `args`: 传递给 `target` 的参数，要使用tuple类型
- `kwargs`: 同上，使用字典类型dict

下面这个例子传递一个数字给线程（这个数字正好等于线程号码），目标函数会打印出这个数字：

```python
import threading

def function(i):
    print ("function called by thread %i\n" % i)
    return

threads = []

for i in range(5):
    t = threading.Thread(target=function , args=(i, ))
    threads.append(t)
    t.start()
    t.join()
```

- 这段代码怀疑存在错误，作者想表达的是线程输出存在无序现象，但因为写了 `t.join()` ，导致t线程结束之前并不会看到后续的线程，换句话说，主线程会调用t线程，然后等待t线程完成再执行for循环开启下一个t线程，事实上，这段代码是顺序运行的，实际运行顺序永远是01234顺序出现，

- 在主程序中，我们使用目标函数 `function` 初始化了一个线程对象 `Thread` 。同时还传入了用于打印的一个参数。
- 线程被创建之后并不会马上运行，需要手动调用 `start()` ， `join()` 让调用它的线程一直等待直到执行结束（即阻塞调用它的主线程， `t` 线程执行结束，主线程才会继续执行）

## 3.2 继承Thread对象

使用threading模块自定义一个新的线程对象，需要下面3步：

- 定义一个 `Thread` 类的子类
- 重写 `__init__(self [,args])` 方法，可以添加额外的参数
- 重写 `run(self, [,args])` 方法来实现线程要做的事情

当创建了新的 `Thread` 子类的时候，你可以实例化这个类，调用 `start()` 方法来启动它。线程启动之后将会执行 `run()` 方法。

```python
import threading
import time

exitFlag = 0

class myThread(threading.Thread):
    def __init__(self,threadID, name, delay):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.delay = delay

    def run(self):
        print("Starting " + self.name)
        print_time(self.name, self.delay, 3)
        print("Exiting " + self.name)

    def print_time(threadName, delay, counter):
        while counter:
            if exitFlag:
                _thread.exit()
            time.sleep(delay)
            print("%s: %s" % (threadName, time.ctime(time.time())))
            counter -= 1

# Create new threads
thread1 = myThread(1, "Thread-1", 1)
thread2 = myThread(2, "Thread-2", 2)

# Start new Threads
thread1.start()
thread2.start()

# Block the main thread
thread1.join()
thread2.join()
print("Exiting Main Thread")
```

每一个自定义线程都通过一个继承 `Thread` 类，重写 `run()` 方法来实现逻辑，这个方法是线程的入口。在主程序中，我们创建了多个 `myThread` 的类型实例，然后执行 `start()` 方法启动它们。调用 `Thread.__init__` 构造器方法是必须的，通过它我们可以给线程定义一些名字或分组之类的属性。调用 `start()` 之后线程变为活跃状态，并且持续直到 `run()` 结束，或者中间出现异常。`join()` 命令控制主线程的中断和重启。所有的线程都执行完成之后，程序结束。如果不使用`join()`则主线程会先于子线程结束，但不影响子线程继续执行。

