#ifndef LEPTRINO__FORCE_TORQUE_SENSOR_HPP_
#define LEPTRINO__FORCE_TORQUE_SENSOR_HPP_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <string>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "leptrino/visibility_control.hpp"
#include <leptrino/pComResInternal.h>
#include <leptrino/pCommon.h>
#include <leptrino/rs_comm.h>

namespace leptrino_constants
{
const double SENSOR_INIT_TIMEOUT_SEC = 2.0;
const double SENSOR_INIT_POLL_DURATION_SEC = 0.01;
const int SENSOR_READ_RATE_HZ = 1200;
}  // namespace leptrino_constants

typedef struct ST_SystemInfo
{
  int com_ok;
} SystemInfo;

namespace leptrino
{

class ForceTorqueSensor
{
public:
  LEPTRINO_FORCE_TORQUE_SENSOR_PUBLIC
  explicit ForceTorqueSensor();

  LEPTRINO_FORCE_TORQUE_SENSOR_PUBLIC
  virtual ~ForceTorqueSensor();

  bool initialize(const std::string & com_port, const rclcpp::Logger & logger = rclcpp::get_logger("leptrino_force_torque_sensor"));

  void serialStart(const rclcpp::Logger & logger = rclcpp::get_logger("leptrino_force_torque_sensor"));

  void serialStop(const rclcpp::Logger & logger = rclcpp::get_logger("leptrino_force_torque_sensor"));

  bool read(geometry_msgs::msg::Wrench & wrench);

private:
  void close();

  void sendData(UCHAR * pucInput, USHORT usSize);

  bool getProductInfo(const rclcpp::Logger & logger = rclcpp::get_logger("leptrino_force_torque_sensor"));

  bool getLimit(const rclcpp::Logger & logger = rclcpp::get_logger("leptrino_force_torque_sensor"));

  SystemInfo gSys;
  UCHAR CommRcvBuff[256];
  UCHAR CommSendBuff[1024];
  UCHAR SendBuff[512];
  double conversion_factor[FN_Num];

  std::string g_com_port;

  bool is_initialized_;
};

}  // namespace leptrino

#endif  // LEPTRINO__FORCE_TORQUE_SENSOR_HPP_
