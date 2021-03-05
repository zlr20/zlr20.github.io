---
title: Python文件与文件夹操作
date: 2021-03-05 08:54:26
categories:
- Python
tags:
- Python
---

日常对于批量处理文件的需求非常多，用Python写脚本可以非常方便地实现。本篇整理了Python中最常用到的文件操作，均采用内置函数实现，无论是批处理还是读取文件都会用到。

<!--More-->

## 1. 遍历文件夹

Path对象的glob方法，能通过正则匹配来遍历文件夹并过滤文件，返回一个生成器。

```python
from pathlib import Path
jpg_files = Path('./train_data').glob("*.jpg")
print(type(jpg_files)) # generator
for jpg_file in jgp_files:
    print(jpg_file)    # train_data/0.jpg ...
    print(jpg_file.name) # 0.jpg ...
```

## 2. 获取文件信息

```python
from pathlib import Path
jpg_file = Path('0.jpg') # 或者由上一节那样遍历生成
print(jpg_file.name) # 文件名+后缀 0.jpg
print(jpg_file.stem) # 文件名 0
print(jpg_file.suffix) # 后缀 .jpg
print(jpg_file.stat()) # 获取文件大小、创建时间等
```

## 3. 创建文件夹

```python
target_folder = Path("./test") # Path("./test/test1/test2")
target_folder.mkdir(parents=True,exist_ok=True)
```

这个好处是：

- 可以递归的建目录，`os`模块中目录是`mkdir`，子目录是`mkdirs`
- 可以指定`exist_ok`参数，从而不必担心目录已经存在而报错

## 4. 删除文件夹

```python
import os
os.system("rm -rf ./test") # 处理非空的最简单办法
```

