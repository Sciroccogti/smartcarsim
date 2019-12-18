# SCS 智能车仿真平台

以全国大学生智能车竞赛为背景，
依照智能车竞赛规则建立仿真模型，旨在为广大参与智能车
竞赛的参赛选手提供一个仿真智能车机械，验证软件控制算
法的平台。

该平台以刚体模型库 OpenDE 和图形库 OpenGL 为基础，
描述并绘制智能车以及赛道的机械模型，仿真智能车行进的
物理过程。


## 版本信息

本发布包为 SCS 的源代码发布包，版本号为 1.0.1 。

本发布包包含SCS-1.0.1的源文件、头文件，ode、freeglut
的库文件、头文件，以及 Linux、VS2008、VS2010三个环境
下的样例工程。样例工程中包含三个组别的样例 AI 决策函
数，以及样例绘制函数。


## 目录结构及说明
```
SCS-1.0.1	根目录
├── scs			SCS 目录
│   ├── include			SCS 头文件目录
│   └── src			SCS 源文件目录
├── ai			样例 AI 目录
│   ├── include			样例 AI 头文目录
│   ├── src			样例 AI 源文目录
│   └── track			赛道文件目录
├── build		编译目录
│   ├── include			编译所需头文件目录
│   ├── linux			Linux 环境编译目录
│   ├── VS2008			VS2008 环境编译目录
│   └── VS2010			VS2010 环境编译目录
├── doc			文档目录
│   ├── INSTALL.TXT		使用方法说明文件
│   ├── LICENSE.TXT		用户协议文件
│   ├── NEWS.TXT		开发新闻文件
│   ├── ChangeLog.TXT		修改日志文件
│   ├── SCS-1.0.1.pdf		使用手册及编程接口
│   └── src			文档源代码目录
└── README.md		本文件
```

### 完整的目录树
```
SCS-1.0.1
├── ai
│   ├── build
│   │   ├── ai
│   │   ├── api.o
│   │   ├── car.o
│   │   ├── draw.o
│   │   ├── libscs.a
│   │   ├── main.o
│   │   ├── Makefile
│   │   ├── simulation.o
│   │   ├── track.o
│   │   ├── ui.o
│   │   └── vector.o
│   ├── include
│   │   ├── scs.h
│   │   └── vector.h
│   ├── src
│   │   └── main.cpp
│   └── track
│       ├── final8.trk
│       └── province8.trk
├── build
│   ├── include
│   │   ├── GL
│   │   │   ├── freeglut_ext.h
│   │   │   ├── freeglut.h
│   │   │   ├── freeglut_std.h
│   │   │   └── glut.h
│   │   └── ode
│   │       ├── collision.h
│   │       ├── collision_space.h
│   │       ├── collision_trimesh.h
│   │       ├── common.h
│   │       ├── compatibility.h
│   │       ├── contact.h
│   │       ├── error.h
│   │       ├── export-dif.h
│   │       ├── mass.h
│   │       ├── matrix.h
│   │       ├── memory.h
│   │       ├── misc.h
│   │       ├── objects.h
│   │       ├── odeconfig.h
│   │       ├── odecpp_collision.h
│   │       ├── odecpp.h
│   │       ├── ode.h
│   │       ├── odeinit.h
│   │       ├── odemath.h
│   │       ├── odemath_legacy.h
│   │       ├── rotation.h
│   │       └── timer.h
│   ├── linux
│   │   ├── ai
│   │   ├── api.o
│   │   ├── car.o
│   │   ├── draw.o
│   │   ├── libscs.a
│   │   ├── main.o
│   │   ├── Makefile
│   │   ├── simulation.o
│   │   ├── track.o
│   │   ├── ui.o
│   │   └── vector.o
│   ├── VS2008
│   │   ├── ai
│   │   │   ├── ai.vcproj
│   │   │   └── lib
│   │   │       ├── freeglut_static.lib
│   │   │       └── ode_double.lib
│   │   ├── scs
│   │   │   └── scs.vcproj
│   │   ├── VS2008.sln
│   │   └── VS2008.suo
│   └── VS2010
│       ├── ai
│       │   ├── ai.vcxproj
│       │   ├── ai.vcxproj.filters
│       │   └── lib
│       │       ├── freeglut_static.lib
│       │       └── ode_double.lib
│       ├── scs
│       │   ├── scs.vcxproj
│       │   └── scs.vcxproj.filters
│       ├── VS2010.sln
│       └── VS2010.suo
├── doc
│   ├── ChangeLog.TXT
│   ├── INSTALL.TXT
│   ├── NEWS.TXT
│   ├── SCS-1.0.1.pdf
│   └── src
│       ├── avatar
│       │   ├── andy.png
│       │   ├── berniw.jpg
│       │   ├── fengzi.png
│       │   ├── free.jpg
│       │   ├── hanquan.jpg
│       │   ├── may.jpg
│       │   ├── Remi-Coulom.jpg
│       │   └── tju_speed.jpg
│       ├── figure
│       │   ├── coordinate_balance.png
│       │   ├── coordinate.png
│       │   ├── demo1.png
│       │   ├── demo2.png
│       │   ├── demo3.png
│       │   ├── logic.pdf
│       │   ├── time.pdf
│       │   └── window.png
│       ├── Makefile
│       └── SCS-1.0.1.tex
├── LICENSE
├── README.md
└── scs
    ├── include
    │   ├── api.h
    │   ├── car.h
    │   ├── common.h
    │   ├── draw.h
    │   ├── key.h
    │   ├── mytime.h
    │   ├── simulation.h
    │   ├── track.h
    │   ├── ui.h
    │   └── vector.h
    └── src
        ├── api.cpp
        ├── car.cpp
        ├── draw.cpp
        ├── simulation.cpp
        ├── track.cpp
        ├── ui.cpp
        └── vector.cpp

25 directories, 108 files
```

## 使用方法

请先阅读 doc/ 目录下的用户协议 LICENSE.TXT ，然后阅读
同目录下的 INSTALL.TXT 文件。


## 联系方式

作者邮箱	yukunlinykl@users.sf.net

中文主页	http://www.smartcarsim.com/

英文主页	http://sourceforge.net/p/smartcarsim/
