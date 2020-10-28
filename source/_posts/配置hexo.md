---
title: 配置hexo
date: 2020-10-27 22:48:26
tags:
- Hexo
categories:
- 博客搭建
---

## 0.什么是Hexo

Hexo 是一个快速、简洁且高效的博客框架。Hexo 使用 Markdown]（或其他渲染引擎）解析文章，在几秒内，即可利用靓丽的主题生成静态网页。

本文介绍了如何在Windows上搭建Hexo框架，并与Github进行关联，实现可以编写与访问的个人博客。

<!-- more -->

## 1.windows上配置必要的环境

- git https://git-scm.com/download/win

  下载安装文件，一路安装即可，开始菜单出现git bash, git gui等

- nodejs https://nodejs.org/en/download/

  下载安装文件，一路安装即可，开始菜单出现Node.js等

- 测试

  <img src='http://i2.tiimg.com/728885/a1d975c8fdd72df2.png' width=450>

## 2.windows上安装Hexo

cmd或powershell执行下载安装命令

```bash
cd XX/XX/MyBlogs # 进入本地博客目录
npm install -g hexo-cli
hexo init
```

## 3.本地测试

```bash
hexo n my_first_blog # 新建一篇文章，n = new
hexo g # 生成，g = generate
hexo s # 静态部署，s = server 可以本地4000端口访问
```

到浏览器里输入http://localhost:4000/查看效果

## 4.主题美化

如果嫌弃主题太丑，可以尝试更换主题，这里使用网上最流行的Next主题

- 将next主题下载并放到 `MyBlogs/themes`目录中，其他主题也同理。

  ```bash
  git clone https://github.com/theme-next/hexo-theme-next themes/next
  ```

- 打开`MyBlogs/_config.yml`将`Themes`设置为next。这是站点配置文件

  同时可以设置站点的名字、作者等

- 再去到`Myblogs/themes/next`去配置具体信息。这是主题配置文件

  同时可以设置外观、背景、图标等

## 5.与Github关联

- 在github创建仓库 `username.github.io`

- 将Git与github绑定

  ```bash
  git config --global user.name "你的GitHub用户名"
  git config --global user.email "你的GitHub注册邮箱"
  ssh-keygen -t rsa -C "你的GitHub注册邮箱"
  ```

  默认回车即可

  <img src='http://i2.tiimg.com/728885/11319686226085a3.png' width=450>

  然后找到刚才生成的.ssh文件夹中的id_rsa.pub密钥，打开，复制内容

- 在github新建new SSH key，将密钥粘贴过去，title任取

  <img src='http://i2.tiimg.com/728885/6968de853cd14be0.png' width=450>

- 在本地git bash验证是否设置公钥成功

  <img src='http://i2.tiimg.com/728885/acb3794c0c928105.png' width=450>

这里之所以设置GitHub密钥原因是，通过非对称加密的公钥与私钥来完成加密，公钥放置在GitHub上，私钥放置在自己的电脑里。GitHub要求每次推送代码都是合法用户，所以每次推送都需要输入账号密码验证推送用户是否是合法用户，为了省去每次输入密码的步骤，采用了ssh，当你推送的时候，git就会匹配你的私钥跟GitHub上面的公钥是否是配对的，若是匹配就认为你是合法用户，则允许推送。这样可以保证每次的推送都是正确合法的。

- 打开站点配置yml文件，将deploy与github关联

  ```
  deploy:
    type: git
    repo: https://github.com/zlr20/zlr20.github.io
    branch: master
  ```

- 最后安装Git部署插件，输入命令：

  ```bash
  npm install hexo-deployer-git --save
  ```

- 部署到github.io上

  ```bash
  hexo clean 
  hexo g 
  hexo d # d = deploy
  ```

- 访问你的博客 https://zlr20.github.io/

## 6.绑定域名（略）

