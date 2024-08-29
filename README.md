# fv_tracking

fly-vision(飞视)目标跟踪代码

## 项目简介

基于ROS(Robot Operating System)框架一个物体追踪的项目，目前处于持续开发阶段

## 环境配置

当前项目的运行环境为Ubuntu 16.04/18.04 LTS + ROS Kinetic/Melodic + OpenCV 3.3.1，请确保环境安装正确



#### 在工作空间的src目录下输入以下命名将所有文件导入

```shell
$ git clone http://gitlab.amovauto.com/amovlab/fv_tracking.git
```

## 运行步骤

### 1 命令行方式

#### 1.1 构建项目

在工作空间目录下使用以下命令

```shell
$ catkin_make
```

#### 1.2 运行

```shell
$ rosrun fv_tracking web_cam
$ rosrun fv_tracking tracker_kcf
```

## 模块介绍

web_cam：从摄像头中读取图片帧，发布图片帧（默认摄像头0，改为其他需要修改源码）

发布（Image话题）：/camera/rgb/image_raw

tracker_kcf：订阅图片帧，可以鼠标设置追踪框，显示实时追踪效果（框选跟踪）

订阅（Image话题）：/camera/rgb/image_raw

发布（Pose2D话题，距离图像中心点的像素误差x,y）：/vision/error_pixels


