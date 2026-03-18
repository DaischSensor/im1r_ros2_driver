#include "im1r_ros2_driver/im1r_driver.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>
#include <cmath>
#include <iostream>
#include <chrono>
#include <algorithm>
#include <iomanip>

namespace im1r_driver
{

// Helper template from user example
template <typename T>
inline T readLE(const uint8_t* p)
{
    T v; std::memcpy(&v, p, sizeof(T)); return v;
}

const uint8_t IM1RDriver::HEAD[2] = {0xA5, 0x5A};
const uint8_t IM1RDriver::TAIL[2] = {0x0D, 0x0A};

IM1RDriver::IM1RDriver(const rclcpp::NodeOptions & options)
: Node("im1r_driver_node", options),
  keep_running_(false),
  serial_fd_(-1)
{
  // Declare parameters
  this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  this->declare_parameter<int>("baud_rate", 115200);
  this->declare_parameter<std::string>("frame_id", "IM1R");

  // Get parameters
  this->get_parameter("serial_port", serial_port_);
  this->get_parameter("baud_rate", baud_rate_);
  this->get_parameter("frame_id", frame_id_);

  // Create publishers
  pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
  pub_temp_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature", 10);
  pub_extra_ = this->create_publisher<im1r_ros2_driver::msg::Im1rExtra>("im1r/extra", 10);

  // Open serial port
  serial_fd_ = open_serial(serial_port_, baud_rate_);
  
  if (serial_fd_ >= 0) {
      RCLCPP_INFO(this->get_logger(), "Serial port %s opened at %d baud", serial_port_.c_str(), baud_rate_);
      keep_running_ = true;
      read_thread_ = std::thread(&IM1RDriver::read_loop, this);
  } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", serial_port_.c_str());
  }
}

IM1RDriver::~IM1RDriver()
{
  keep_running_ = false;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  close_serial();
}

int IM1RDriver::open_serial(const std::string& dev, int baud)
{
    int fd = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        RCLCPP_ERROR(this->get_logger(), "open(%s) failed: %s", dev.c_str(), strerror(errno));
        return -1;
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "tcgetattr failed: %s", strerror(errno));
        close(fd);
        return -1;
    }

    cfmakeraw(&tty); // Raw mode: no echo, no signals, no canonical processing

    speed_t baud_speed;
    switch (baud) {
        case 9600: baud_speed = B9600; break;
        case 19200: baud_speed = B19200; break;
        case 38400: baud_speed = B38400; break;
        case 57600: baud_speed = B57600; break;
        case 115200: baud_speed = B115200; break;
        case 230400: baud_speed = B230400; break;
        case 460800: baud_speed = B460800; break;
        case 921600: baud_speed = B921600; break;
        default: 
            RCLCPP_WARN(this->get_logger(), "Unsupported baudrate %d, defaulting to 115200", baud);
            baud_speed = B115200; 
            break;
    }

    cfsetispeed(&tty, baud_speed);
    cfsetospeed(&tty, baud_speed);

    tty.c_cflag |= (CLOCAL | CREAD);   // Enable receiver, ignore modem control
    tty.c_cflag &= ~CRTSCTS;           // Disable hardware flow control
    
    // Set timeout
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1; // 0.1s timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "tcsetattr failed: %s", strerror(errno));
        close(fd);
        return -1;
    }
    
    // Flush buffer
    tcflush(fd, TCIOFLUSH);

    return fd;
}

void IM1RDriver::close_serial()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

uint8_t IM1RDriver::crc8(const uint8_t* buf, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= buf[i];
        for (int j = 0; j < 8; ++j) {
            crc = (crc & 0x80) ? uint8_t((crc << 1) ^ 0x07)
                               : uint8_t(crc << 1);
        }
    }
    return crc;
}

bool IM1RDriver::parse_frame(const uint8_t* f, FrameData& out)
{
    // Check header (already checked by caller but double check safe)
    if (std::memcmp(f, HEAD, HEAD_LEN)) return false;
    
    // Check LEN
    if (f[HEAD_LEN + DOM_LEN + CMD_LEN] != DATA_LEN) return false; // f[4] == 64
    
    // Check Tail
    if (f[FRAME_LEN-2] != TAIL[0] || f[FRAME_LEN-1] != TAIL[1]) return false;
    
    // Check CRC
    size_t crc_off = HEAD_LEN + DOM_LEN + CMD_LEN + LEN_LEN + DATA_LEN; // 2+1+1+1+64 = 69
    // CRC is at index 69 (0-based)
    if (crc8(f, crc_off) != f[crc_off]) {
        RCLCPP_DEBUG(this->get_logger(), "CRC mismatch");
        return false;
    }
    
    // Parse
    const uint8_t* d = f + HEAD_LEN + DOM_LEN + CMD_LEN + LEN_LEN; // Data starts at index 5
    out.count        = d[0];
    out.timestamp_ms = readLE<uint64_t>(d+1);
    for (int i=0;i<3;++i) out.acc[i]  = readLE<float>(d+ 9 +4*i);
    for (int i=0;i<3;++i) out.gyro[i] = readLE<float>(d+21 +4*i);
    for (int i=0;i<3;++i) out.att[i]  = readLE<float>(d+33 +4*i);
    for (int i=0;i<4;++i) out.quat[i] = readLE<float>(d+45 +4*i);
    out.temperature  = readLE<int16_t>(d+61)*0.1f;
    out.imu_status   = d[63];
    
    return true;
}

void IM1RDriver::read_loop()
{
    std::vector<uint8_t> buf;
    buf.reserve(4096);
    uint8_t tmp[256];
    
    RCLCPP_INFO(this->get_logger(), "Starting serial read loop");

    while (rclcpp::ok() && keep_running_) {
        if (serial_fd_ < 0) {
            // Try to reopen
            std::this_thread::sleep_for(std::chrono::seconds(1));
            serial_fd_ = open_serial(serial_port_, baud_rate_);
            if (serial_fd_ >= 0) {
                 RCLCPP_INFO(this->get_logger(), "Reopened serial port");
            }
            continue;
        }

        ssize_t n = ::read(serial_fd_, tmp, sizeof(tmp));
        if (n > 0) {
            buf.insert(buf.end(), tmp, tmp + n);
            
            // Search for frames
            while (true) {
                // Find Header
                auto it = std::search(buf.begin(), buf.end(), HEAD, HEAD + HEAD_LEN);
                if (it == buf.end()) {
                    // No header found, keep some bytes just in case split header
                    if (buf.size() >= HEAD_LEN) {
                         buf.erase(buf.begin(), buf.end() - (HEAD_LEN - 1));
                    }
                    break; 
                }
                
                size_t pos = std::distance(buf.begin(), it);
                
                // Check if we have enough data for a full frame
                if (buf.size() - pos < FRAME_LEN) {
                    // Remove data before header
                    if (pos > 0) {
                        buf.erase(buf.begin(), buf.begin() + pos);
                    }
                    break; // Wait for more data
                }
                
                // Parse frame
                const uint8_t* frame_ptr = &buf[pos];
                FrameData parsed{};
                if (parse_frame(frame_ptr, parsed)) {
                    // Success
                    publish_data(parsed);
                    
                    // Remove this frame
                    buf.erase(buf.begin(), buf.begin() + pos + FRAME_LEN);
                } else {
                    // Failed to parse (CRC error or invalid fields)
                    // Skip header and search again
                    RCLCPP_DEBUG(this->get_logger(), "Frame parse failed at pos %zu", pos);
                    buf.erase(buf.begin(), buf.begin() + pos + 1);
                }
            }
        } else if (n < 0) {
             if (errno != EAGAIN && errno != EWOULDBLOCK) {
                 RCLCPP_WARN(this->get_logger(), "Serial read error: %s", strerror(errno));
             }
             std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } else {
             // n == 0
             std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

void IM1RDriver::publish_data(const FrameData& data)
{
    auto stamp = this->now();
    
    // Publish IMU
    auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
    imu_msg->header.stamp = stamp;
    imu_msg->header.frame_id = frame_id_;
    
    imu_msg->linear_acceleration.x = data.acc[0];
    imu_msg->linear_acceleration.y = data.acc[1];
    imu_msg->linear_acceleration.z = data.acc[2];
    
    // Convert deg/s to rad/s
    constexpr float DEG_TO_RAD = M_PI / 180.0f;
    imu_msg->angular_velocity.x = data.gyro[0] * DEG_TO_RAD;
    imu_msg->angular_velocity.y = data.gyro[1] * DEG_TO_RAD;
    imu_msg->angular_velocity.z = data.gyro[2] * DEG_TO_RAD;
    
    const float qw = data.quat[0];
    const float qx = data.quat[1];
    const float qy = data.quat[2];
    const float qz = data.quat[3];

    const float norm2 = qw * qw + qx * qx + qy * qy + qz * qz;
    constexpr float kQuatNormEps2 = 1e-12f;
    if (norm2 < kQuatNormEps2) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "IMU quaternion is near-zero (%.3e). IMU may be not initialized or abnormal.",
            static_cast<double>(norm2));
    }

    imu_msg->orientation.w = qw;
    imu_msg->orientation.x = qx;
    imu_msg->orientation.y = qy;
    imu_msg->orientation.z = qz;
    
    pub_imu_->publish(std::move(imu_msg));
    
    // Publish Temperature
    auto temp_msg = std::make_unique<sensor_msgs::msg::Temperature>();
    temp_msg->header.stamp = stamp;
    temp_msg->header.frame_id = frame_id_;
    temp_msg->temperature = data.temperature;
    pub_temp_->publish(std::move(temp_msg));
    
    // Publish Extra
    auto extra_msg = std::make_unique<im1r_ros2_driver::msg::Im1rExtra>();
    extra_msg->count = data.count;
    extra_msg->timestamp = data.timestamp_ms;
    extra_msg->pitch = data.att[0];
    extra_msg->roll = data.att[1];
    extra_msg->yaw = data.att[2];
    extra_msg->imu_status = data.imu_status;

    extra_msg->gyro_bias_x = 0.0;
    extra_msg->gyro_bias_y = 0.0;
    extra_msg->gyro_bias_z = 0.0;
    extra_msg->gyro_static_bias_x = 0.0;
    extra_msg->gyro_static_bias_y = 0.0;
    extra_msg->gyro_static_bias_z = 0.0;
    
    pub_extra_->publish(std::move(extra_msg));
}

} // namespace im1r_driver
