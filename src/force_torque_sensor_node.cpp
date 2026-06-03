#include "leptrino/force_torque_sensor_node.hpp"

#include <chrono>
#include <functional>

#include <rclcpp_components/register_node_macro.hpp>

namespace leptrino
{

ForceTorqueSensorNode::ForceTorqueSensorNode(const rclcpp::NodeOptions & options)
: Node("leptrino_force_torque_sensor", options),
  kFrameId_(this->declare_parameter<std::string>("frame_id", "leptrino")),
  new_data_available_(false),
  is_initialized_(false),
  shutdown_requested_(false)
{
  std::string com_port = this->declare_parameter("com_port", "/dev/ttyACM0");
  double rate_hz = this->declare_parameter("rate", 1200.0);

  if (rate_hz <= 0) {
    RCLCPP_WARN(this->get_logger(), "Parameter 'rate' must be positive. Defaulting to 1.0 Hz.");
    rate_hz = 1.0;
  }

  if (!sensor_.initialize(com_port, this->get_logger())) {
    RCLCPP_FATAL(
      this->get_logger(),
      "\033[31m Failed to initialize sensor on com port %s. Shutting down. \033[0m",
      com_port.c_str());
    rclcpp::shutdown();
    return;
  }

  std::string topic_name =
    std::string(this->get_namespace()) + "/" + std::string(this->get_name()) + "/wrench";
  wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("topic_name", 10);

  publish_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / rate_hz),
    std::bind(&ForceTorqueSensorNode::timerCallback, this));

  is_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "Leptrino node started. Publishing at %.1f Hz.", rate_hz);

  start();
}

ForceTorqueSensorNode::~ForceTorqueSensorNode()
{
  shutdown_requested_ = true;

  if (is_initialized_) {
    sensor_.serialStop(this->get_logger());
  }

  if (sensor_thread_.joinable()) {
    sensor_thread_.join();
  }
}

void ForceTorqueSensorNode::start()
{
  if (!is_initialized_) {
    RCLCPP_ERROR(
      this->get_logger(), "Node was not initialized correctly. Aborting sensor thread start.");
    return;
  }

  sensor_.serialStart(this->get_logger());

  sensor_thread_ = std::thread([this]() {
    rclcpp::Rate read_rate(leptrino_constants::SENSOR_READ_RATE_HZ);
    auto last_read_time = std::chrono::steady_clock::now();

    while (rclcpp::ok() && !shutdown_requested_) {
      geometry_msgs::msg::Wrench temp_wrench;
      if (sensor_.read(temp_wrench)) {
        {
          std::lock_guard<std::mutex> lock(wrench_mutex_);
          latest_wrench_ = temp_wrench;
          new_data_available_ = true;
        }
      } else {
        {
          std::lock_guard<std::mutex> lock(wrench_mutex_);
          new_data_available_ = false;
        }
      }

      auto current_time = std::chrono::steady_clock::now();
      double actual_period = std::chrono::duration<double>(current_time - last_read_time).count();
      last_read_time = current_time;

      RCLCPP_DEBUG(
        this->get_logger(), "Sensor read loop period: %.6f s (%.1f Hz)", actual_period,
        1.0 / actual_period);

      read_rate.sleep();
    }
  });
}

void ForceTorqueSensorNode::timerCallback()
{
  if (!new_data_available_) {
    static rclcpp::Time last_warn_time = this->now();

    if ((this->now() - last_warn_time).seconds() >= 1.0) {
      RCLCPP_WARN(this->get_logger(), "No new data is available from the sensor to publish.");
      last_warn_time = this->now();
    }
    return;
  }

  geometry_msgs::msg::WrenchStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = kFrameId_;

  {
    std::lock_guard<std::mutex> lock(wrench_mutex_);
    msg.wrench = latest_wrench_;
  }

  wrench_pub_->publish(msg);
}

}  // namespace leptrino

RCLCPP_COMPONENTS_REGISTER_NODE(leptrino::ForceTorqueSensorNode)
