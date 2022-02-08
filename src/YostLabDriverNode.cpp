#include "YostLabDriver.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("imu");
  rclcpp::spin(std::make_shared<YostLabDriver>());
  rclcpp::shutdown();
  
  // ros::NodeHandle nh;
  // ros::NodeHandle priv_nh("~");
  // YostLabDriver imu_drv(nh, priv_nh);
  // imu_drv.run();

  return 0;
}
