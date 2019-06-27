# PointCloudViewer

#### Description
深圳智绘科技研发的一款无人机实时控制、激光点云显示软件。

A software for UAV real-time control and laser point cloud display.

![PointCloudViewer](/tools/PointCloudViewer.png)



#### Software Architecture
软件由C++开发，利用Qt搭建图形界面，使用三维渲染引擎OSG来进行渲染。

无人机与控制软件使用局域网通信，点云数据由draco压缩后使用udp传输。

The software is developed by C++, using Qt for GUI and OSG for rendering, and the point cloud data is compressed by draco and transmitted by udp.

#### Prerequisites
1. OpenSceneGraph 3.2
2. Qt version 5.5 or newer
3. draco
4. libssh
5. ...

  Make sure you have a C++11 compliant compiler (gcc 5.5+ / clang)

#### Instructions
1. Clone the project
2. Build
3. Run with -h
