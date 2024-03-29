#pragma once

#include <malloc.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <functional>
#include <serial/serial.h>

#include <memory>

using namespace serial;

class SerialInterface
{
private:
  using SerialPtr = std::unique_ptr<serial::Serial>;
  // baudrate
  int baud_;
  // connection related variables
  SerialPtr connection_port_;
  std::string port_;
  bool connected_ = false;
  // logger zone
  const std::string log_zone_;

public:

  void setSerialNode(rclcpp::Node &serial_node_)
  {
    serial_node_.declare_parameter<int>("BAUD_RATE", 115200);
    serial_node_.get_parameter("BAUD_RATE", baud_);
    serial_node_.declare_parameter<std::string>("SERIAL_PORT", "/dev/imu_top");
    serial_node_.get_parameter("SERIAL_PORT", port_);
  }

  const int &getBaudRate()
  {
    return baud_;
  }

  /**
   * Destructor for the interface
   */
  const std::string &getSerialPort()
  {
    return port_;
  }

  /**
   * Returns true if connection is established with the serial port.
   *
   * @return true if connected to serial, false otherwise
   */
  bool isConnected()
  {
    return connected_;
  }

  /**
   * Destructor
   */
  ~SerialInterface()
  {
    if (connection_port_ != NULL)
    {
      if (connection_port_->isOpen())
      {
        // RCLCPP_INFO(local_serial_node_.get_logger(log_zone_), " Closing the Serial Port: %s", port_);
        connection_port_->close();
        connected_ = false;
      }
    }
  }

  /**
   * Connects to the serial port.
   */
  void serialConnect()
  {
    try
    {
      const auto uint_baud = static_cast<uint32_t>(baud_);
      const auto timeout = Timeout::simpleTimeout(60000);  // timeout in milliseconds
      connection_port_.reset(new Serial(port_, baud_, timeout));
    }
    catch (IOException &e)
    {
      std::string ioerror = e.what();
      // RCLCPP_ERROR(local_serial_node_.get_logger(log_zone_), "Unable to connect port: %s", port_.c_str());
      // RCLCPP_ERROR(local_serial_node_.get_logger(log_zone_), "Is the serial port open? : %s", ioerror.c_str());
      connected_ = false;
    }

    if (connection_port_ && connection_port_->isOpen())
    {
      // RCLCPP_INFO(local_serial_node_.get_logger(log_zone_), "Connection Established with Port: %s with baudrate: %d", port_.c_str(), baud_);
      connected_ = true;
    }
  }

  /**
   * Writes a string to the serial port
   *
   * @param str The string to write to the serial port
   */
  inline void serialWriteString(const std::string &str)
  {
    connection_port_->write(str);
  }

  /**
   * Sets the timeout of read and write operations.
   *
   * @param timeout The desired timeout in milliseconds
   */
  void setTimeout(int timeout = 500)
  {
    Timeout t = Timeout::simpleTimeout((uint32_t)timeout);
    connection_port_->setTimeout(t);
  }

  /**
   * Gets the timeout of read operations.
   *
   * @return the current timeout of read operations in milliseconds
   */
  int getTimeout()
  {
    Timeout t = connection_port_->getTimeout();
    return t.read_timeout_constant;
  }

  /**
   * Flushes the read and write string buffers.
   */
  inline void flush()
  {
    connection_port_->flush();
  }

  /**
   * Flushes the read and write string buffers.
   *
   * @return the string read in from serial
   */
  inline std::string serialReadLine()
  {
    std::string str = connection_port_->readline();
    return str;
  }

  /**
   * Gets the number of available bytes
   *
   * @return the number of available bytes
   */
  inline size_t available()
  {
    return connection_port_->available();
  }
};
