#ifndef IM1R_DRIVER_HPP_
#define IM1R_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <im1r_ros2_interface/msg/im1r_extra.hpp>

#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <optional>
#include <array>

// Serial headers
#include <termios.h>

namespace im1r_driver
{

// Data structure from user example
struct FrameData {
    uint8_t  count{};          // Frame count
    uint64_t timestamp_ms{};   // UNIX ms timestamp
    float    acc[3]{};         // Acc X/Y/Z (m/s^2)
    float    gyro[3]{};        // Gyro X/Y/Z (deg/s)
    float    att[3]{};         // Euler Pitch/Roll/Yaw (deg)
    float    quat[4]{};        // Quat (w,x,y,z)
    float    temperature{};    // Temp (C)
    uint8_t  imu_status{};     // Status
};

class IM1RDriver : public rclcpp::Node
{
public:
  explicit IM1RDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~IM1RDriver();

private:
  // Parameters
  std::string serial_port_;
  int baud_rate_;
  std::string frame_id_;

  // Serial port file descriptor
  int serial_fd_;

  // Thread for reading serial data
  std::thread read_thread_;
  std::atomic<bool> keep_running_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_temp_;
  rclcpp::Publisher<im1r_ros2_interface::msg::Im1rExtra>::SharedPtr pub_extra_;

  // Methods
  int open_serial(const std::string& dev, int baud);
  void close_serial();
  void read_loop();
  
  // Helpers
  std::optional<FrameData> parse_frame(const uint8_t* f);
  uint8_t crc8(const uint8_t* buf, size_t len);
  
  void publish_data(const FrameData& data);

  // Constants
  static constexpr uint8_t HEAD[2] = {0xA5, 0x5A};
  static constexpr uint8_t TAIL[2] = {0x0D, 0x0A};
  
  static constexpr size_t HEAD_LEN = 2;
  static constexpr size_t DOM_LEN  = 1;
  static constexpr size_t CMD_LEN  = 1;
  static constexpr size_t LEN_LEN  = 1;
  static constexpr size_t DATA_LEN = 64;
  static constexpr size_t CRC_LEN  = 1;
  static constexpr size_t TAIL_LEN = 2;
  static constexpr size_t FRAME_LEN = HEAD_LEN + DOM_LEN + CMD_LEN + LEN_LEN + 
                                      DATA_LEN + CRC_LEN + TAIL_LEN; // = 72
};

} // namespace im1r_driver

#endif // IM1R_DRIVER_HPP_
