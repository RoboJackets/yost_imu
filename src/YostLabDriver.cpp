#include "YostLabDriver.hpp"
#include "SerialInterface.hpp"
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

using namespace std::chrono_literals;

YostLabDriver::YostLabDriver() : Node("IMU"), SerialInterface()
{
  this->setSerialNode(*this);
  serialConnect();
  diagnostic_updater::Updater update(this);
  this->updater = std::make_unique<diagnostic_updater::Updater>(this);
  updater->setHardwareIDf("IMU: %s", getSerialPort().c_str());
  updater->add("IMU Diagnostic", this, &YostLabDriver::imu_diagnostic);

  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
  magnet_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/imu_mag", 10);

  // use identity matrix as default orientation correction
  this->declare_parameter<std::vector<double>>("imu_orientation_correction", { 1, 0, 0, 0, 1, 0, 0, 0, 1 });
  this->get_parameter("imu_orientation_correction", imu_orientation_correction_);
  this->declare_parameter<double>("orientation_rotation", 0.0);
  this->get_parameter("orientation_rotation", orientation_rotation_);
  this->get_parameter("frame_id", frame_id_);
  this->declare_parameter<bool>("calibrate_imu", false);
  this->get_parameter("calibrate_imu", calibrate_imu_);

}

//! Destructor
YostLabDriver::~YostLabDriver()
{
  updater->broadcast(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "IMU Node Terminated");
}

void YostLabDriver::restoreFactorySettings()
{
  serialWriteString(RESTORE_FACTORY_SETTINGS);
}

std::string YostLabDriver::getSoftwareVersion()
{
  flush();
  serialWriteString(GET_FIRMWARE_VERSION_STRING);
  const std::string buf = serialReadLine();
  RCLCPP_INFO(this->get_logger(), "Software version: %s", buf.c_str());
  return buf;
}

void YostLabDriver::imu_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (sensor_temp_ > MAX_IMU_TEMP)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "IMU temp too high");
  }
  else if (sensor_temp_ < MIN_IMU_TEMP)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "IMU temp too low");
  }
  else if ((this->get_clock()->now() - lastUpdateTime_).seconds() > IMU_TIMEOUT_DELAY)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "IMU not updating");
  }
  else if (abs(quaternion_length_ - 1.0) > QUATERNION_LENGTH_TOL)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "IMU quaternion isn't normalized");
  }
  else
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "IMU Online");
  }
  stat.add("software_version", software_version_);
  stat.add("calibration_mode", calibration_mode_);
  stat.add("mi_mode", mi_mode_);
  stat.add("axis_direction", axis_direction_);
  stat.add("imu_temp", sensor_temp_);
  double roll, pitch, yaw;
  tf2::Matrix3x3(last_quat_).getRPY(roll, pitch, yaw);
  double radian_to_degrees = 180.0 / M_PI;
  stat.add("roll", roll * radian_to_degrees);
  stat.add("pitch", pitch * radian_to_degrees);
  stat.add("yaw", yaw * radian_to_degrees);
}

std::string YostLabDriver::getEulerDecomp()
{
  flush();
  serialWriteString(GET_EULER_DECOMPOSTION_ORDER);
  const std::string buf = serialReadLine();
  const std::string ret_buf = [&]() {
    if (buf == "0\r\n")
      return "XYZ";
    else if (buf == "1\r\n")
      return "YZX";
    else if (buf == "2\r\n")
      return "ZXY";
    else if (buf == "3\r\n")
      return "ZYX";
    else if (buf == "4\r\n")
      return "XZY";
    else if (buf == "5\r\n")
      return "YXZ";
    else
      return "Unknown";
  }();
  RCLCPP_INFO(this->get_logger(), "Euler Decomposition: %s, buf is: %s", ret_buf.c_str(), buf.c_str());
  return ret_buf;
}

std::string YostLabDriver::getAxisDirection()
{
  flush();
  serialWriteString(GET_AXIS_DIRECTION);
  const std::string buf = serialReadLine();
  const std::string ret_buf = [&]() {
    if (buf == "0\r\n")
      return "X: Right, Y: Up, Z: Forward";
    else if (buf == "1\r\n")
      return "X: Right, Y: Forward, Z: Up";
    else if (buf == "2\r\n")
      return "X: Up, Y: Right, Z: Forward";
    else if (buf == "3\r\n")
      return "X: Forward, Y: Right, Z: Up";
    else if (buf == "4\r\n")
      return "X: Up, Y: Forward, Z: Right";
    else if (buf == "5\r\n")
      return "X: Forward, Y: Up, Z: Right";
    else
      return "Unknown";
  }();
  RCLCPP_INFO(this->get_logger(), "Axis Direction: %s, buf is: %s", ret_buf.c_str(), buf.c_str());
  return ret_buf;
}

void YostLabDriver::startGyroCalibration()
{
  flush();
  RCLCPP_INFO(this->get_logger(), "Starting Auto Gyro Calibration");
  serialWriteString(BEGIN_GYRO_AUTO_CALIB);
  rclcpp::sleep_for(std::chrono::seconds(5));
}

void YostLabDriver::setMIMode(bool on)
{
  if (on)
    serialWriteString(SET_MI_MODE_ENABLED);
  else
    serialWriteString(SET_MI_MODE_DISABLED);
}

std::string YostLabDriver::getCalibMode()
{
  flush();
  serialWriteString(GET_CALIB_MODE);
  const std::string buf = serialReadLine();
  const std::string ret_buf = [&]() {
    if (buf == "0\r\n")
      return "Bias";
    else if (buf == "1\r\n")
      return "Scale and Bias";
    else
      return "Unknown";
  }();
  RCLCPP_INFO(this->get_logger(), "Calibration Mode: %s, buf is: %s", ret_buf.c_str(), buf.c_str());
  return ret_buf;
}

std::string YostLabDriver::getMIMode()
{
  flush();
  serialWriteString(GET_MI_MODE_ENABLED);
  const std::string buf = serialReadLine();
  const std::string ret_buf = [&]() {
    if (buf == "0\r\n")
      return "Disabled";
    else if (buf == "1\r\n")
      return "Enabled";
    else
      return "Unknown";
  }();
  RCLCPP_INFO(this->get_logger(), "MI Mode: %s, buf is: %s", ret_buf.c_str(), buf.c_str());
  return ret_buf;
}

//! Run the serial sync
void YostLabDriver::run()
{
  setAndCheckIMUSettings();

  // Performs auto-gyroscope calibration. Sensor should remain still while samples are taken.
  if (calibrate_imu_)
  {
    startGyroCalibration();
  }

  // commit settings and start streaming
  flush();
  serialWriteString(SET_STREAMING_TIMING_5_MS);
  serialWriteString(START_STREAMING);

}

void YostLabDriver::run2()
{
  int line_num_ = 0;
  std::vector<double> parsed_val;

  while (rclcpp::ok())
  {
    while (available() > 0)
    {
      line_num_ += 1;
      // Parses the line and
      auto terms_count = addToParsedVals(serialReadLine(), parsed_val);
      // Checks for a single a line containing a single number (IMU temp) which signifies the end of the imu message
      if (terms_count == 1)
      {
        // Verify that it is a complete message of 5 lines
        if (line_num_ == 5)
        {
          createAndPublishIMUMessage(parsed_val);
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "Incomplete message from IMU. Throwing it away.");
        }
        parsed_val.clear();
        line_num_ = 0;
      }
    }
  }
}

void YostLabDriver::setAndCheckIMUSettings()
{
  flush();
  setTimeout(500);
  serialWriteString(SET_AXIS_DIRECTIONS);
  serialWriteString(SET_STREAMING_SLOTS);
  serialWriteString(COMMIT_SETTINGS);

  // small delay to allow imu to commit its settings before we read them back
  rclcpp::sleep_for(std::chrono::milliseconds(2));

  // print/debug statements
  software_version_ = getSoftwareVersion();
  axis_direction_ = getAxisDirection();
  std::string euler = getEulerDecomp();
  calibration_mode_ = getCalibMode();
  mi_mode_ = getMIMode();
}

void YostLabDriver::createAndPublishIMUMessage(std::vector<double> &parsed_val)
{
  // orientation correction matrices in 3x3 row-major format and quaternion
  static const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> correction_mat(imu_orientation_correction_.data());
  static const Eigen::Quaternion<double> correction_mat_quat(correction_mat);
  static tf2::Quaternion rot;
  rot.setRPY(0, 0, orientation_rotation_);

  sensor_msgs::msg::Imu imu_msg;
  // imu_msg.header.seq = msg_counter_;
  imu_msg.header.stamp = this->get_clock()->now();
  imu_msg.header.frame_id = frame_id_;
  sensor_msgs::msg::MagneticField magnet_msg;
  // magnet_msg.header.seq = msg_counter_;
  magnet_msg.header.stamp = this->get_clock()->now();
  magnet_msg.header.frame_id = frame_id_;

  // construct quaternion with (x,y,z,w)
  tf2::Quaternion quat{ parsed_val[0], parsed_val[1], parsed_val[2], parsed_val[3] };
  quaternion_length_ = tf2::length(quat);
  quat = rot * quat;

  // the tf::Quaternion has a method to access roll pitch and yaw
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  // the found angles are written in a geometry_msgs::Vector3
  geometry_msgs::msg::Vector3 rpy;
  rpy.x = roll;
  rpy.y = pitch;
  rpy.z = yaw;

  geometry_msgs::msg::Quaternion geometry_quaternion;
  geometry_quaternion.x = quat.getX();
  geometry_quaternion.y = quat.getY();
  geometry_quaternion.z = quat.getZ();
  geometry_quaternion.w = quat.getW();


  // Filtered orientation estimate
  imu_msg.orientation_covariance = { .1, 0, 0, 0, .1, 0, 0, 0, .1 };

  // Corrected angular velocity.
  Eigen::Vector3d angular_vel_raw(parsed_val[4], parsed_val[5], parsed_val[6]);
  angular_vel_raw = correction_mat * angular_vel_raw;

  // Corrected linear acceleration.
  Eigen::Vector3d linear_accel_raw(parsed_val[7], parsed_val[8], parsed_val[9]);
  linear_accel_raw = correction_mat * linear_accel_raw * GRAVITY;

  // Corrected magnetometer
  Eigen::Vector3d compass_raw(parsed_val[10], parsed_val[11], parsed_val[12]);
  compass_raw = correction_mat * compass_raw * GAUSSTOTESLA;

  imu_msg.angular_velocity.x = angular_vel_raw.x();
  imu_msg.angular_velocity.y = angular_vel_raw.y();
  imu_msg.angular_velocity.z = angular_vel_raw.z();
  imu_msg.angular_velocity_covariance = { .1, 0, 0, 0, .1, 0, 0, 0, .07 };

  imu_msg.linear_acceleration.x = linear_accel_raw.x();
  imu_msg.linear_acceleration.y = linear_accel_raw.y();
  imu_msg.linear_acceleration.z = linear_accel_raw.z();
  imu_msg.linear_acceleration_covariance = { .1, 0, 0, 0, .1, 0, 0, 0, .1 };

  magnet_msg.magnetic_field.x = compass_raw.x();
  magnet_msg.magnetic_field.y = compass_raw.y();
  magnet_msg.magnetic_field.z = compass_raw.z();
  magnet_msg.magnetic_field_covariance = { 1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6 };

  sensor_temp_ = parsed_val[13];

  imu_pub_->publish(imu_msg);
  magnet_pub_->publish(magnet_msg);  // Published in teslas
  lastUpdateTime_ = this->get_clock()->now();
  last_quat_ = quat;
  msg_counter_++;
}

int YostLabDriver::addToParsedVals(const std::string &buf, std::vector<double> &parsed_vals)
{
  std::stringstream ss(buf);
  int terms_count = 0;
  double i;
  while (ss >> i)
  {
    parsed_vals.push_back(i);
    terms_count++;
    if (ss.peek() == ',')
      ss.ignore();
  }
  return terms_count;
}
