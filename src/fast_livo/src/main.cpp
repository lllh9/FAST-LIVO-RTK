#include "optimization.h"
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Parameters are loaded from the YAML passed via --params-file. Declaring
  // them automatically from the overrides means readParameters()/the vikit
  // camera loader can simply get_parameter(...) them (mirrors ROS 1 rosparam).
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("laserMapping", options);

  image_transport::ImageTransport it(node);

  // FAST-LIVO2 保持高频局部前端；GTSAM iSAM2 在独立线程执行 RTK、相对位姿
  // 和 Scan Context/GICP 回环的在线全局优化，不会阻塞这里的前端循环。
  LIVMapper mapper(node);
  optimization opti(node);

  mapper.initializeSubscribersAndPublishers(it);
  mapper.run();   // 5 kHz 前端循环，同时用 spin_some 驱动轻量 ROS 回调。

  rclcpp::shutdown();
  return 0;
}
