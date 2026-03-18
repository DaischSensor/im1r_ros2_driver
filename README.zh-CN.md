<div align="center">

[![DAISCH Banner](documentation/README.assets/DAISCH_IM1R_Banner.png)](http://www.daisch.com)


# IM1R_ROS2_Driver

[![GitHub issues](https://img.shields.io/github/issues/DAISCHSensor/im1r_ros2_driver?style=flat-square)](https://github.com/DAISCHSensor/im1r_ros2_driver/issues)
[![GitHub pr](https://img.shields.io/github/issues-pr/DAISCHSensor/im1r_ros2_driver?style=flat-square)](https://github.com/DAISCHSensor/im1r_ros2_driver/pulls)
[![GitHub](https://img.shields.io/github/license/DAISCHSensor/im1r_ros2_driver?style=flat-square)]()
[![GitHub release (with filter)](https://img.shields.io/github/v/release/DAISCHSensor/im1r_ros2_driver?style=flat-square)]()
[![GitHub Repo stars](https://img.shields.io/github/stars/DAISCHSensor/im1r_ros2_driver?style=flat-square)]()

[English](./README.md) · **简体中文** · [Global Official Site](https://www.daischsensor.com) · [中文官网](https://www.daisch.com)

</div>

---

## 目录

- [IM1R\_ROS2\_Driver](#im1r_ros2_driver)
  - [目录](#目录)
  - [项目描述](#项目描述)
  - [入门指南](#入门指南)
    - [系统要求](#系统要求)
    - [安装步骤](#安装步骤)
  - [使用说明](#使用说明)
  - [发布的话题](#发布的话题)
  - [参数介绍](#参数介绍)
    - [标准话题](#标准话题)
      - [imu/data](#imudata)
    - [自定义话题](#自定义话题)
      - [im1r/extra](#im1rextra)
  - [贡献](#贡献)
  - [许可证](#许可证)

## 项目描述

本项目旨在开发和维护适用于 IM1R 产品的 ROS2 驱动程序。

## 入门指南

### 重要提醒（迁移说明）

⚠️ 本驱动已从 Python 版本重构为 C++ 版本，原先独立的自定义消息包 `im1r_ros2_interface` 已合并到本仓库/软件包中。

- 可执行文件名称变化：`im1r_node`（Python）→ `im1r_driver_node`（C++）
- 话题变化：
  - 不再发布 `/temperature`（温度信息合并在 `im1r/extra` 中）
- 如果你是从旧的 Python 版本升级过来，为避免运行到旧节点/旧产物，请清理并重新编译工作空间。

### 系统要求

- Ubuntu 22.04 / ROS2 Humble
- Ubuntu 20.04 / ROS2 Foxy
- 支持 C++14 的编译器（必需）

### 安装步骤

1. 安装 ROS2：
   请参考 [ROS2文档](https://docs.ros.org/en/humble/index.html) 获取详细说明。

2. 创建 ROS2 工作空间：

   ```shell
   mkdir -p ~/ros2_ws/src
   ```
   
3. 克隆项目仓库到 src 目录：

   ```shell
   cd ~/ros2_ws/src
   git clone https://github.com/DAISCHSensor/im1r_ros2_driver.git
   ```
   
4. 安装 ROS 依赖项：

   ```shell
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```
   如果系统中没有安装 `rosdep`，请先安装并更新：
   ```shell
   sudo apt update
   sudo apt install python3-rosdep
   rosdep update
   ```
   
5. 构建工作空间：

   ```shell
   cd ~/ros2_ws/
   colcon build --packages-select im1r_ros2_driver
   ```

6. 添加工作空间的环境变量到 `.bashrc`：

   ⚠️ **注意：**如果之前已经在 .bashrc 中添加过以下内容，请不要重复添加，以免出现重复加载或配置混乱。
   
   对于 ROS2  Foxy
   
   ```shell
   echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
   echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```
   
   对于 ROS2  Humble
   
   ```shell
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

## 使用说明

1. 通过UART1数据线束连接IM1R

2. 识别 IM1R 设备的串口：

   ``` shell
   sudo dmesg | grep tty
   ```
   
   假设 IM1R 设备连接到 /dev/ttyUSB0：

   ``` shell
   sudo chmod 666 /dev/ttyUSB0
   ```

3. 确认 IM1R 设备的波特率，默认波特率是115200，可通过上位机 DS_RVision 来更改

4. 启动驱动节点：

   - 假设当前IM1R连接的串口是 `/dev/ttyUSB0`
   - 假设当前IM1R使用的波特率是 `115200`

   **方法 1: 使用 ros2 run**
   ``` shell
   ros2 run im1r_ros2_driver im1r_driver_node --ros-args -p serial_port:=/dev/ttyUSB0 -p baud_rate:=115200
   ```

   **方法 2: 使用 launch 文件**
   ``` shell
   ros2 launch im1r_ros2_driver im1r_driver.launch.py serial_port:=/dev/ttyUSB0 baud_rate:=115200 frame_id:=IM1R
   ```
   可用参数：`serial_port`、`baud_rate`、`frame_id`。
   如有需要，也可以直接修改 [im1r_driver.launch.py](file:///home/daisch/ros2_ws/src/im1r_ros2_driver/launch/im1r_driver.launch.py) 中的默认参数。

5. 列出所有话题：

   ``` shell
   ros2 topic list
   ```

6. 输出指定话题的内容：

   ``` shell
   ros2 topic echo /imu/data
   ```


## 发布的话题

- `imu/data` ([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)) 四元数、角速度和线性加速度
- `im1r/extra` ([DAISCH 自定义话题](#custom-topic)) 来自 **IM1R** 的额外参数

## 参数介绍

### 标准话题

#### imu/data

| Variable                                     | Supported |
| -------------------------------------------- | --------- |
| time `header.stamp`                          | ✔️        |
| string `header.frame_id`                     | ✔️        |
| float64 `orientation.x`                      | ✔️        |
| float64 `orientation.y`                      | ✔️        |
| float64 `orientation.z`                      | ✔️        |
| float64 `orientation.w`                      | ✔️        |
| float64[9] `orientation_covariance`          | ✘        |
| float64 `angular_velocity.x`                 | ✔️        |
| float64 `angular_velocity.y`                 | ✔️        |
| float64 `angular_velocity.z`                 | ✔️        |
| float64[9] `angular_velocity_covariance`     | ✘        |
| float64 `linear_acceleration.x`              | ✔️        |
| float64 `linear_acceleration.y`              | ✔️        |
| float64 `linear_acceleration.z`              | ✔️        |
| float64[9] `linear_acceleration_covariance`  | ✘        |

### 自定义话题

#### im1r/extra

| Variable                   | Type       | Definition                                | Unit              | Remarks                                             |
| -------------------------- | ---------- | ----------------------------------------- | ----------------- | --------------------------------------------------- |
| `count`                    | uint8      | 消息计数器                                | -                 | 0~255 循环递增                                      |
| `timestamp`                | uint64     | 测量时间戳                                | microseconds (µs) | UNIX 时间                                           |
| `pitch`                    | float64    | 俯仰角（Pitch）                           | degrees (°)       |                                                     |
| `roll`                     | float64    | 横滚角（Roll）                            | degrees (°)       |                                                     |
| `yaw`                      | float64    | 偏航角（Yaw）                             | degrees (°)       |                                                     |
| `imu_status`               | uint8      | IMU 状态                                  | -                 | Bit 定义请参考产品手册。                            |
| `temperature`              | float64    | 温度                                      | degrees Celsius (°C) |                                                  |


## 贡献

<a href="https://github.com/DAISCHSensor/im1r_ros2_driver/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=DAISCHSensor/im1r_ros2_driver"/>
</a>

## 许可证

[BSD-3-Clause](./LICENSE)
