#ifndef LEPTRINO__FORCE_TORQUE_SENSOR_NODE_HPP_
#define LEPTRINO__FORCE_TORQUE_SENSOR_NODE_HPP_

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "leptrino/force_torque_sensor.hpp"
#include "leptrino/visibility_control.hpp"

namespace leptrino
{

class ForceTorqueSensorNode : public rclcpp::Node
{
public:
  LEPTRINO_FORCE_TORQUE_SENSOR_PUBLIC
  explicit ForceTorqueSensorNode(const rclcpp::NodeOptions & options);
  virtual ~ForceTorqueSensorNode();

private:
  void start();

  void timerCallback();

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  ForceTorqueSensor sensor_;

  geometry_msgs::msg::Wrench latest_wrench_;
  std::mutex wrench_mutex_;
  const std::string kFrameId_;
  bool new_data_available_;
  bool is_initialized_;

  std::thread sensor_thread_;
  std::atomic<bool> shutdown_requested_;
};

}  // namespace leptrino

#endif
