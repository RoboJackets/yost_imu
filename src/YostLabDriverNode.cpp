#include "YostLabDriver.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<YostLabDriver>();
  
  node->run();
  double spin_frequency_ = 100.0;
  rclcpp::Rate loop_rate(spin_frequency_); // Hz
  while(rclcpp::ok())
  {
    node->run2();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
