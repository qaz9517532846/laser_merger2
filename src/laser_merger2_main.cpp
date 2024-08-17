#include <laser_merger2/laser_merger2.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  InitGPUDev();
  rclcpp::spin(std::make_shared<laser_merger2>());
  rclcpp::shutdown();
  return 0;
}