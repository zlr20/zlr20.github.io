---
title: next主题美化
date: 2020-10-28 09:19:31
mathjax: true
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



## 渲染数学公式

目前，NexT 提供两种数学公式渲染引擎，分别为 [MathJax](https://www.mathjax.org/) 和 [Katex](https://khan.github.io/KaTeX/)。如果你选择使用 MathJax 进行数学公式渲染，你需要使用 [hexo-renderer-pandoc](https://github.com/wzpan/hexo-renderer-pandoc) 或者 [hexo-renderer-kramed](https://github.com/sun11/hexo-renderer-kramed) （不推荐）作为 Hexo 的 Markdown 渲染器。

首先，卸载原有的渲染器 `hexo-renderer-marked`，并安装这两种渲染器的**其中一个**：

```
npm uninstall hexo-renderer-marked
npm install hexo-renderer-pandoc # 或者 hexo-renderer-kramed
```

然后在 `next/_config.yml` 中将 `mathjax` 的 `enable` 打开。

```yml
# Math Formulas Render Support
math:
  # Default (true) will load mathjax / katex script on demand.
  # That is it only render those page which has `mathjax: true` in Front-matter.
  # If you set it to false, it will load mathjax / katex srcipt EVERY PAGE.
  per_page: True

  # hexo-renderer-pandoc (or hexo-renderer-kramed) required for full MathJax support.
  mathjax:
    enable: True
    # See: https://mhchem.github.io/MathJax-mhchem/
    mhchem: True
```

`per_page`为`true` 或者 `false`，默认为 `true`。这个选项是控制是否在每篇文章都渲染数学公式。

默认(`true`) 的行为是**只对 Front-matter 中含有 `mathjax: true` 的文章进行数学公式渲染**。

如果 Front-matter 中不含有 `mathjax: true`，或者 `mathjax: false`，那么 NexT 将不会对这些文章进行数学公式渲染。

例如：

```
<!-- 这篇文章会渲染数学公式 -->
---
title: 'Will Render Math'
mathjax: true
---
....
<!-- 这篇文章不会渲染数学公式 -->
---
title: 'Not Render Math'
mathjax: false
---
....
<!-- 这篇文章也不会渲染数学公式 -->
---
title: 'Not Render Math Either'
---
....
```

当你将它设置为 `false` 时，它就会在每个页面都加载 MathJax 来进行数学公式渲染。

执行 Hexo 生成，部署，或者启动服务器：

```
hexo clean && hexo g -d
# 或者 hexo clean && hexo s
```