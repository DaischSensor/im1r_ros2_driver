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
      - [temperature](#temperature)
    - [自定义话题](#自定义话题)
      - [im1r/extra](#im1rextra)
  - [贡献](#贡献)
  - [许可证](#许可证)

## 项目描述

本项目旨在开发和维护适用于 IM1R 产品的 ROS2 驱动程序。

## 入门指南

### 系统要求

- Ubuntu 22.04 / ROS2 Humble
- Ubuntu 20.04 / ROS2 Foxy

### 安装步骤

1. 安装 ROS2：
   请参考 [ROS2文档](https://docs.ros.org/en/humble/index.html) 获取详细说明。

2. 安装依赖项：

   运行以下命令安装依赖项：

   ```shell
   sudo apt update
   sudo apt install python3-pip
   pip3 install pyserial
   ```

3. 创建 ROS2 工作空间：

   ```shell
   mkdir -p ~/ros2_ws/src
   ```
   
4. 克隆项目仓库到 src 目录：

   ```shell
   cd ~/ros2_ws/src
   git clone https://github.com/DAISCHSensor/im1r_ros2_driver.git
   git clone https://github.com/DAISCHSensor/im1r_ros2_interface.git
   ```
   
5. 安装 ROS 依赖项：

   ```shell
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```
   
6. 构建工作空间：

   ```shell
   cd ~/ros2_ws/
   colcon build
   ```

7. 添加工作空间的环境变量到 `.bashrc`：

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

   ``` shell
   ros2 run im1r_ros2_driver im1r_node --ros-args -p serial_port:=/dev/ttyUSB0 -p baud_rate:=115200
   ```

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
- `temperature` ([sensor_msgs/Temperature](http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html)) 来自设备的温度
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

#### temperature

| Variable                                     | Supported |
| -------------------------------------------- | --------- |
| time `header.stamp`                          | ✔️        |
| string `header.frame_id`                     | ✔️        |
| float64 `temperature`                        | ✔️        |
| float64 `variance`                           | ✘        |

### 自定义话题

#### im1r/extra

| Variable                   | Type       | Definition                                | Unit              | Remarks                                             |
| -------------------------- | ---------- | ----------------------------------------- | ----------------- | --------------------------------------------------- |
| `count`                    | uint8      | Message counter                           | -                 | 0~255 cyclic increment                              |
| `timestamp`                | uint64     | Timestamp of the measurement              | microseconds (µs) | UNIX time                                           |
| `pitch`                    | float64    | Pitch angle                               | degrees (°)       |                                                     |
| `roll`                     | float64    | Roll angle                                | degrees (°)       |                                                     |
| `yaw` | float64 | Yaw angle | degrees (°) | |
| `imu_status`               | uint8      | IMU status indicator                      | -                 | Bit 0: Acceleration valid (0) / invalid (1)<br>Bit 2: Angular velocity valid (0) / invalid (1)<br>Higher bits are not defined   |
| `gyro_bias_x`              | float64    | Gyroscope bias along the X axis           | radians/second (rad/s) |                                                 |
| `gyro_bias_y`              | float64    | Gyroscope bias along the Y axis           | radians/second (rad/s) |                                                 |
| `gyro_bias_z`              | float64    | Gyroscope bias along the Z axis           | radians/second (rad/s) |                                                 |
| `gyro_static_bias_x`       | float64    | Static gyroscope bias along the X axis    | radians/second (rad/s) |                                                 |
| `gyro_static_bias_y`       | float64    | Static gyroscope bias along the Y axis    | radians/second (rad/s) |                                                 |
| `gyro_static_bias_z`       | float64    | Static gyroscope bias along the Z axis    | radians/second (rad/s) |                                                 |


## 贡献

<a href="https://github.com/DAISCHSensor/im1r_ros2_driver/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=DAISCHSensor/im1r_ros2_driver"/>
</a>

## 许可证

[BSD-3-Clause](./LICENSE)
