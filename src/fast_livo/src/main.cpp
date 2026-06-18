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

  // Front-end (ESIKF LIVO) and back-end (GTSAM/RTK batch optimizer) share one
  // node and communicate in-process over topics, exactly as upstream.
  LIVMapper mapper(node);
  optimization opti(node);

  mapper.initializeSubscribersAndPublishers(it);
  mapper.run();   // 5 kHz busy loop; spins the node via rclcpp::spin_some

  rclcpp::shutdown();
  return 0;
}
