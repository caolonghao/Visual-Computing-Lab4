# 可视计算概论 Lab1 Base Code

## 安装

本项目目前支持Windows和MacOS，不支持Linux。

### 1.安装依赖

本项目需要以下工具构建：

* [git](http://git-scm.com/)：版本管理工具
* C++编译器：Windows下推荐使用Visual Studio（MSVC），MacOS使用XCode（Apple Clang）
* [xmake](https://xmake.io/#/)：项目构建工具，参考[官方文档](https://xmake.io/#/guide/installation)安装

### 2.编译

编译命令：
```
xmake f -m release
xmake
```
* 重复编译出错时可以使用`xmake clean`清空缓存
* `xmake f -m debug`可以切换到debug模式，切换之后需要运行`xmake -r`重新编译
* xmake会自动下载所需要的第三方库文件并链接到项目中，如果下载过程中遇到网络问题，有如下解决方式：
  - 可运行`xmake g --proxy_pac=github_mirror.lua`将github.com重定向到hub.fastgit.xyz
  - 可运行`xmake g --pkg_searchdirs=<download-dir>`并根据报错提示手动下载
  - 打开本地代理，使用命令行设置好环境变量`HTTPS_PROXY="127.0.0.1:<port>"`，之后在命令行中运行`xmake`

### 3.运行

运行命令：
```
xmake run
```
编译生成的二进制程序在`bin`目录下，也可以进入`bin`目录下手动执行

### 使用IDE

我们推荐大家使用`VSCode`、`Vim`、`Emacs`等代码编辑工具编辑代码，使用`xmake project -k compile_commands`为代码编辑器添加Intellisense支持，然后在命令行中使用`xmake`构建系统。如果想要使用IDE编译和测试代码，除了上面介绍的流程外也可以使用xmake构建IDE项目：
* 对于Windows系统，运行`xmake project -k vsxmake`会在vsxmake20xx目录下生成Visual Studio的项目
* 对于Mac系统，运行`xmake project -k xcode`可以生成Xcode项目
* 对于Clion等其他IDE，运行`xmake project -k cmake`生成cmake项目并使用IDE中的cmake功能