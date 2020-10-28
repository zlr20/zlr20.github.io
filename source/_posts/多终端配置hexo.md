---
title: 多终端配置hexo
date: 2020-10-28 13:09:43
tags:
- Hexo
categories:
- 博客搭建
---

机制是这样的，由于`hexo d`上传部署到github的其实是hexo编译后的文件，是用来生成网页的，不包含源文件。可以看到，并没有source等源文件在内。也就是上传的是在本地目录里自动生成的`.deploy_git`里面。其他文件 ，包括我们写在source 里面的，和配置文件，主题文件，都没有上传到github。所以可以利用git的分支管理，将源文件上传到github的另一个分支即可。

<!--More-->

## 1.github上的设置

- 首先，在github上新建一个hexo分支

- 在这个仓库的settings中，选择默认分支为hexo分支（这样每次同步的时候就不用指定分支，比较方便）

  <img src='http://i1.fuimg.com/728885/6aea68a321528a33.png' width=400>

- 将本地的文件上传到hexo分支

- 注意.gitignore中的是不用上传的

  ```
  .DS_Store
  Thumbs.db
  db.json
  *.log
  node_modules/
  public/
  .deploy*/
  ```

至此，我们就可以在新的终端上pull源文件，并push修改，通过hexo分支，实现同步。当然我们的`hexo d`还是会部署在master分支上，这是在站点yml中声明的。

## 2.在deepin上配置git

```bash
sudo apt install git
git config --global usr.name XXXX
git config --global usr.email XXXX@XXXX
ssh-keygen -t rsa -C XXXX@XXXX
```

<img src='http://i2.tiimg.com/728885/bf0120b40a221288.png' width=500>

在github上新建ssh key，将生成的密钥复制过去。

<img src='http://i2.tiimg.com/728885/8b9102b6d40c0c98.png' width=500>

测试连接：

```bash
ssh -T git@github.com
```

## 3.在deepin上配置nodejs和npm

千万不要`sudo apt install nodejs`，版本只有4.x且装不上npm

直接官网下载最新的二进制包，并配置bashrc。参考https://blog.csdn.net/qq_41897021/article/details/107720553

测试：

<img src='http://i2.tiimg.com/728885/6da6881b4e3373d0.png' width=500>

## 4.安装hexo

```bash
npm install hexo-cli -g
```

## 5.从github上克隆hexo分支到本地

```bash
git clone git@github.com:zlr20/zlr20.github.io XXX
npm install # 主要是node_modules/内容
npm install hexo-deployer-git --save
```

## 6.编写完成后上传

```bash
# 部署到master
hexo d
# 上传源码到hexo分支
git add .
git commit -m "your comment"
git push
```

## 7.下次写前同步

```bash
git pull
```

