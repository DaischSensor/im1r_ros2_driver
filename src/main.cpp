#include "im1r_ros2_driver/im1r_driver.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<im1r_driver::IM1RDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
