<div align="center">

[![DAISCH Banner](documentation/README.assets/DAISCH_IM1R_Banner.png)](http://www.daisch.com)


# IM1R_ROS2_Driver

[![GitHub issues](https://img.shields.io/github/issues/DAISCHSensor/im1r_ros2_driver?style=flat-square)](https://github.com/DAISCHSensor/im1r_ros2_driver/issues)
[![GitHub pr](https://img.shields.io/github/issues-pr/DAISCHSensor/im1r_ros2_driver?style=flat-square)](https://github.com/DAISCHSensor/im1r_ros2_driver/pulls)
[![GitHub](https://img.shields.io/github/license/DAISCHSensor/im1r_ros2_driver?style=flat-square)]()
[![GitHub release (with filter)](https://img.shields.io/github/v/release/DAISCHSensor/im1r_ros2_driver?style=flat-square)]()
[![GitHub Repo stars](https://img.shields.io/github/stars/DAISCHSensor/im1r_ros2_driver?style=flat-square)]()

**English** · [简体中文](./README.zh-CN.md) · [Global Official Site](https://www.daischsensor.com) · [中文官网](https://www.daisch.com)

</div>

---

## Table of Contents

- [IM1R\_ROS\_Driver](#im1r_ros_driver)
  - [Table of Contents](#table-of-contents)
  - [Project Description](#project-description)
  - [Getting Started](#getting-started)
    - [System Requirements](#system-requirements)
    - [Installation Setups](#installation-setups)
  - [Usage Instructions](#usage-instructions)
  - [Published Topics](#published-topics)
  - [Parameters Introductions](#parameters-introductions)
    - [Standard Topic](#standard-topic)
      - [imu/data](#imudata)
      - [temperature](#temperature)
    - [Custom Topic](#custom-topic)
      - [im1r/extra](#im1rextra)
  - [Contributing](#contributing)
  - [License](#license)

## Project Description

This project aims to develop and maintain ROS2 drivers for the IM1R product.

## Getting Started

### System Requirements

- Ubuntu 22.04 / ROS2 Humble

### Installation Setups

1. Install ROS2:
   Please refer to the [ROS2 Documentation](https://docs.ros.org/en/humble/index.html) for detailed instructions.

2. Install Dependencies:

      Run the following commands to install dependencies based on your system's Python version:
      
        ```shell
        sudo apt update
        sudo apt install python3-pip
        pip3 install pyserial
        ```

3. Create ROS2 workspace:

   ``` shell
   mkdir -p ~/ros2_ws/src
   ```

4. Clone the project repository to the src directory of your catkin workspace:

   ``` shell
   cd ~/ros2_ws/src
   git clone https://github.com/DAISCHSensor/im1r_ros2_driver.git
   git clone https://github.com/DAISCHSensor/im1r_ros2_interface.git
   ```
   
5. Build the driver:

   ``` shell
   cd ~/ros2_ws/
   colcon build
   ```

6. Update the `.bashrc` file:

   ``` shell
   echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

## Usage Instructions

1. Connect IM1R via UART1 data cable.

2. Identify the serial port for the IM1R device:

   ``` shell
   sudo dmesg | grep tty
   ```

3. Set the serial port permissions:
   Assuming the IM1R device is connected to /dev/ttyUSB0:

   ``` shell
   sudo chmod 666 /dev/ttyUSB0
   ```

4. Launch the driver node:

   ``` shell
   ros2 run im1r_ros2_driver im1r_node --ros-args -p serial_port:=/dev/ttyUSB0 -p baud_rate:=115200
   ```

   - `/dev/ttyUSB0` is the serial port.
   - `115200` is the baud rate used by IM1R, adjust it as needed.

5. List all the topic:

   ``` shell
   ros2 topic list
   ```

6. echo the specific topic:

   ``` shell
   ros2 topic echo /imu/data
   ```

## Published Topics

- `imu/data` ([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)) quaternion, angular velocity and linear acceleration
- `temperature` ([sensor_msgs/Temperature](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)) temperature from device
- `im1r/extra` ([DAISCH Custom Topic](#custom-topic)) extra params from **IM1R**

## Parameters Introductions

### Standard Topic

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

### Custom Topic

#### im1r/extra

| Variable                   | Type       | Definition                                | Unit              | Remarks                                             |
| -------------------------- | ---------- | ----------------------------------------- | ----------------- | --------------------------------------------------- |
| `count`                    | uint8      | Message counter                           | -                 | 0~255 cyclic increment                              |
| `timestamp`                | uint64     | Timestamp of the measurement              | microseconds (µs) | UNIX time                                           |
| `pitch`                    | float64    | Pitch angle                               | degrees (°)       |                                                     |
| `roll`                     | float64    | Roll angle                                | degrees (°)       |                                                     |
| `imu_status`               | uint8      | IMU status indicator                      | -                 | Bit 0: Acceleration valid (0) / invalid (1)<br>Bit 2: Angular velocity valid (0) / invalid (1)<br>Higher bits are not defined   |
| `gyro_bias_x`              | float64    | Gyroscope bias along the X axis           | radians/second (rad/s) |                                                 |
| `gyro_bias_y`              | float64    | Gyroscope bias along the Y axis           | radians/second (rad/s) |                                                 |
| `gyro_bias_z`              | float64    | Gyroscope bias along the Z axis           | radians/second (rad/s) |                                                 |
| `gyro_static_bias_x`       | float64    | Static gyroscope bias along the X axis    | radians/second (rad/s) |                                                 |
| `gyro_static_bias_y`       | float64    | Static gyroscope bias along the Y axis    | radians/second (rad/s) |                                                 |
| `gyro_static_bias_z`       | float64    | Static gyroscope bias along the Z axis    | radians/second (rad/s) |                                                 |


## Contributing

<a href="https://github.com/DAISCHSensor/im1r_ros2_driver/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=DAISCHSensor/im1r_ros2_driver"/>
</a>

## License

[BSD-3-Clause](./LICENSE)