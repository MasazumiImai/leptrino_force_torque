#include "leptrino/force_torque_sensor_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;

  const auto leptrino_force_torque_sensor =
    std::make_shared<leptrino::ForceTorqueSensorNode>(rclcpp::NodeOptions());
  exec.add_node(leptrino_force_torque_sensor);
  exec.spin();

  rclcpp::shutdown();
}
