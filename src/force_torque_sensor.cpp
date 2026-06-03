#include "leptrino/force_torque_sensor.hpp"

#include <chrono>
#include <cstring>
#include <thread>

namespace leptrino
{

ForceTorqueSensor::ForceTorqueSensor() : is_initialized_(false)
{
  memset(conversion_factor, 0, sizeof(conversion_factor));
  gSys.com_ok = NG;
}

ForceTorqueSensor::~ForceTorqueSensor()
{
  close();
}

bool ForceTorqueSensor::initialize(const std::string & com_port, const rclcpp::Logger & logger)
{
  if (Comm_Open(com_port.c_str()) != OK) {
    return false;
  }

  Comm_Setup(460800, PAR_NON, BIT_LEN_8, 0, 0, CHR_ETX);
  gSys.com_ok = OK;

  if (!getProductInfo(logger)) {
    RCLCPP_ERROR(logger, "Failed to get sensor product info.");
    return false;
  }

  if (!getLimit(logger)) {
    RCLCPP_ERROR(logger, "Failed to get sensor limit.");
    return false;
  }

  is_initialized_ = true;

  return true;
}

void ForceTorqueSensor::serialStart(const rclcpp::Logger & logger)
{
  USHORT len = 0x04;             // Data length

  RCLCPP_DEBUG(logger, "Start sensor.");
  SendBuff[0] = len;             // Length
  SendBuff[1] = 0xFF;            // Sensor No.
  SendBuff[2] = CMD_DATA_START;  // Command type
  SendBuff[3] = 0;               // Reserve

  sendData(SendBuff, len);
}

void ForceTorqueSensor::serialStop(const rclcpp::Logger & logger)
{
  USHORT len = 0x04;            // Data length

  RCLCPP_DEBUG(logger, "Stop sensor.");
  SendBuff[0] = len;            // Length
  SendBuff[1] = 0xFF;           // Sensor No.
  SendBuff[2] = CMD_DATA_STOP;  // Command type
  SendBuff[3] = 0;              // Reserve

  sendData(SendBuff, len);
}

bool ForceTorqueSensor::read(geometry_msgs::msg::Wrench & wrench)
{
  Comm_Rcv();

  if (Comm_CheckRcv() != 0) {
    memset(CommRcvBuff, 0, sizeof(CommRcvBuff));

    if (Comm_GetRcvData(CommRcvBuff) > 0) {
      ST_R_DATA_GET_F * stForce = (ST_R_DATA_GET_F *)CommRcvBuff;

      wrench.force.x = stForce->ssForce[0] * conversion_factor[0];
      wrench.force.y = stForce->ssForce[1] * conversion_factor[1];
      wrench.force.z = stForce->ssForce[2] * conversion_factor[2];
      wrench.torque.x = stForce->ssForce[3] * conversion_factor[3];
      wrench.torque.y = stForce->ssForce[4] * conversion_factor[4];
      wrench.torque.z = stForce->ssForce[5] * conversion_factor[5];

      return true;
    }
  }

  return false;
}

void ForceTorqueSensor::close()
{
  if (is_initialized_ && gSys.com_ok == OK) {
    Comm_Close();
    gSys.com_ok = NG;
    is_initialized_ = false;
  }
}

void ForceTorqueSensor::sendData(UCHAR * pucInput, USHORT usSize)
{
  USHORT usCnt;
  UCHAR ucWork;
  UCHAR ucBCC = 0;
  UCHAR * pucWrite = &CommSendBuff[0];
  USHORT usRealSize;

  *pucWrite = CHR_DLE;
  pucWrite++;
  *pucWrite = CHR_STX;
  pucWrite++;
  usRealSize = 2;

  for (usCnt = 0; usCnt < usSize; usCnt++) {
    ucWork = pucInput[usCnt];
    if (ucWork == CHR_DLE) {
      *pucWrite = CHR_DLE;
      pucWrite++;
      usRealSize++;
    }
    *pucWrite = ucWork;
    ucBCC ^= ucWork;
    pucWrite++;
    usRealSize++;
  }

  *pucWrite = CHR_DLE;
  pucWrite++;
  *pucWrite = CHR_ETX;
  ucBCC ^= CHR_ETX;
  pucWrite++;
  *pucWrite = ucBCC;
  usRealSize += 3;

  Comm_SendData(&CommSendBuff[0], usRealSize);

  // return OK;
}

bool ForceTorqueSensor::getProductInfo(const rclcpp::Logger & logger)
{
  USHORT len = 0x04;

  RCLCPP_DEBUG(logger, "Get sensor information.");
  SendBuff[0] = len;          // Length
  SendBuff[1] = 0xFF;         // Sensor No.
  SendBuff[2] = CMD_GET_INF;  // Command type
  SendBuff[3] = 0;            // Reserve

  sendData(SendBuff, len);

  auto start_time = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration<double>(current_time - start_time).count();
    if (elapsed_time >= leptrino_constants::SENSOR_INIT_TIMEOUT_SEC) {
      break;
    }

    Comm_Rcv();
    if (Comm_CheckRcv() != 0 && Comm_GetRcvData(CommRcvBuff) > 0) {
      ST_R_GET_INF * stGetInfo = (ST_R_GET_INF *)CommRcvBuff;

      stGetInfo->scFVer[F_VER_SIZE - 1] = '\0';
      RCLCPP_DEBUG(logger, "  - Version: %s", stGetInfo->scFVer);

      stGetInfo->scSerial[SERIAL_SIZE - 1] = '\0';
      RCLCPP_DEBUG(logger, "  - SerialNo: %s", stGetInfo->scSerial);

      stGetInfo->scPName[P_NAME_SIZE - 1] = '\0';
      RCLCPP_DEBUG(logger, "  - Type: %s", stGetInfo->scPName);

      return true;
    }

    std::this_thread::sleep_for(
      std::chrono::duration<double>(leptrino_constants::SENSOR_INIT_POLL_DURATION_SEC));
  }

  return false;
}

bool ForceTorqueSensor::getLimit(const rclcpp::Logger & logger)
{
  USHORT len = 0x04;

  RCLCPP_DEBUG(logger, "Get sensor limit");
  SendBuff[0] = len;            // Length
  SendBuff[1] = 0xFF;           // Sensor No.
  SendBuff[2] = CMD_GET_LIMIT;  // Command type
  SendBuff[3] = 0;              // Reserve

  sendData(SendBuff, len);

  auto start_time = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration<double>(current_time - start_time).count();
    if (elapsed_time >= leptrino_constants::SENSOR_INIT_TIMEOUT_SEC) {
      break;
    }

    Comm_Rcv();
    if (Comm_CheckRcv() != 0 && Comm_GetRcvData(CommRcvBuff) > 0) {
      ST_R_LEP_GET_LIMIT * stGetLimit = (ST_R_LEP_GET_LIMIT *)CommRcvBuff;
      for (int i = 0; i < FN_Num; i++) {
        conversion_factor[i] = stGetLimit->fLimit[i] * 1e-4;
      }

      RCLCPP_DEBUG(
        logger, "Sensor limit values received successfully.");

      return true;
    }
    std::this_thread::sleep_for(
      std::chrono::duration<double>(leptrino_constants::SENSOR_INIT_POLL_DURATION_SEC));
  }

  return false;
}

}  // namespace leptrino
