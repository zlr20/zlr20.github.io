---
title: next主题美化
date: 2020-10-28 09:19:31
tags:
- Hexo
categories:
- 博客搭建
---

主题美化(持续更新ing)

<!-- more -->

## 修改next主题

```yaml
# Schemes
#scheme: Muse
#scheme: Mist
#scheme: Pisces
scheme: Gemini
```

## 修改站点图标

```yaml
favicon:
  small: /images/small.ico
  medium: /images/medium.ico
  #apple_touch_icon: /images/apple-touch-icon-next.png
  #safari_pinned_tab: /images/logo.svg
  #android_manifest: /images/manifest.json
  #ms_browserconfig: /images/browserconfig.xml
```

可以去https://www.bitbug.net/ 制作。小图标16×16，中等图标32×32。

## 修改菜单选项

```yaml
menu:
  home: / || fa fa-home
  about: /about/ || fa fa-user
  tags: /tags/ || fa fa-tags
  categories: /categories/ || fa fa-th
  archives: /archives/ || fa fa-archive
  #schedule: /schedule/ || fa fa-calendar
  #sitemap: /sitemap.xml || fa fa-sitemap
  #commonweal: /404/ || fa fa-heartbeat

# Enable / Disable menu icons / item badges.
menu_settings:
  icons: true
  badges: false
```

图标直接从https://fontawesome.com/icons 上引就好。

###### about页（关于）

```bash
hexo new page about
```

在index.md文件中写About页面的内容。

###### categories页（分类）

```
hexo new page categories
```

找到`index.md`这个文件，添加`type: "categories"`到内容中，添加后是这样的：

```
---
title: 文章分类
date: 2017-05-27 13:47:40
type: "categories"
---
```

保存并关闭文件。

###### tags页（标签）

同分类页，`type: "tags"`

