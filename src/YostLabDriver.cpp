// #pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include "SerialInterface.hpp"

#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>
#include <diagnostic_updater/diagnostic_updater.hpp>
// #include <diagnostic_updater/publisher.hpp>

#include <memory>

using namespace std::chrono_literals;

// This is the  basic ros-based device driver of IMU
class YostLabDriver : public rclcpp::Node, SerialInterface
{
  
public:
  //! constructor and destructor
  explicit YostLabDriver(const rclcpp::NodeOptions& options) : rclcpp::Node("yost_imu_driver", options)
  {
    YostLabDriver();
  }

  explicit YostLabDriver() : rclcpp::Node("yost_imu_driver")
  {
    this->setSerialNode(*this);
    serialConnect();
    diagnostic_updater::Updater update(this);
    this->updater = std::make_unique<diagnostic_updater::Updater>(this);
    updater->setHardwareIDf("IMU: %s", getSerialPort().c_str());
    updater->add("IMU Diagnostic", this, &YostLabDriver::imu_diagnostic);

    // // imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10); // commenting for error
    // // magnet_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/imu_mag", 10); // commenting for error

    // // use identity matrix as default orientation correction
    // // assertions::param(yostlab_priv_nh_, "imu_orientation_correction", imu_orientation_correction_,
    //                   // std::vector<double>{ 1, 0, 0, 0, 1, 0, 0, 0, 1 });
    // this->declare_parameter<std::vector<double>>("imu_orientation_correction", { 1, 0, 0, 0, 1, 0, 0, 0, 1 });
    // this->get_parameter("imu_orientation_correction", imu_orientation_correction_);
    // // assertions::param(yostlab_priv_nh_, "orientation_rotation", orientation_rotation_, 0.0);
    // this->declare_parameter<double>("orientation_rotation", 0.0);
    // this->get_parameter("orientation_rotation", orientation_rotation_);
    // // assertions::getParam(yostlab_priv_nh_, "frame_id", frame_id_);
    // this->get_parameter("frame_id", frame_id_);
    // // yostlab_priv_nh_.param("spin_frequency", spin_frequency_, 100.0);
    // // this->declare_parameter<double>("spin_frequency", 100.0);
    // // this->get_parameter("spin_frequency", spin_frequency_);
    // // assertions::param(yostlab_priv_nh_, "calibrate_imu", calibrate_imu_, false);
    // this->declare_parameter<bool>("calibrate_imu", false);
    // this->get_parameter("calibrate_imu", calibrate_imu_);
  }

  ~YostLabDriver()
  {
    updater->broadcast(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "IMU Node Terminated");
  }
  //!
  //! \brief run: runs system
  //!
  void run()
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
  void run2()
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
  //!
  //! \brief getSoftwareVersion
  //! \return returns software string version
  //!
  std::string getSoftwareVersion()
  {
    flush();
    serialWriteString(GET_FIRMWARE_VERSION_STRING);
    const std::string buf = serialReadLine();
    RCLCPP_INFO(this->get_logger(), "Software version: %s", buf.c_str());
    return buf;
  }
  //!
  //! \brief restoreFactorySettings resets everything
  //!
  void restoreFactorySettings()
  {
    serialWriteString(RESTORE_FACTORY_SETTINGS);
  }
  //!
  //! \brief imu_diagnostic runs diagnostics
  //!
  void imu_diagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    if (sensor_temp_ > MAX_IMU_TEMP)
    {
      // stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "IMU temp too high"); // commenting for error
    }
    else if (sensor_temp_ < MIN_IMU_TEMP)
    {
      // stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "IMU temp too low"); // commenting for error
    }
    else if ((this->get_clock()->now() - lastUpdateTime_).seconds() > IMU_TIMEOUT_DELAY)
    {
      // stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "IMU not updating"); // commenting for error
    }
    else if (abs(quaternion_length_ - 1.0) > QUATERNION_LENGTH_TOL)
    {
      // stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "IMU quaternion isn't normalized"); // commenting for error
    }
    else
    {
      // stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "IMU Online"); // commenting for error
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
  //!
  //! \brief getAxisDirection
  //! \return returns axis directions
  //!
  std::string getAxisDirection()
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
  //!
  //! \brief getEulerDecomp
  //! \return decompisition string
  //!
  std::string getEulerDecomp()
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
  //!
  //! \brief getCalibMode
  //! \return 0 for bias 1 for scale and bias
  //!
  std::string getCalibMode()
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
  //!
  //! \brief getMIMode
  //! \return 1 if enabled 0 if disabled
  //!
  std::string getMIMode()
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
  //!
  //! \brief startGyroCalibration
  //!
  void startGyroCalibration()
  {
    flush();
    RCLCPP_INFO(this->get_logger(), "Starting Auto Gyro Calibration");
    serialWriteString(BEGIN_GYRO_AUTO_CALIB);
    rclcpp::sleep_for(std::chrono::seconds(5));
  }
  //!
  //! \brief setMIMode
  //! \param on True to set , False to Unset
  //!
  void setMIMode(bool on)
  {
    if (on)
      serialWriteString(SET_MI_MODE_ENABLED);
    else
      serialWriteString(SET_MI_MODE_DISABLED);
  }
  //!
  //! \brief setAndCheckIMUSettings
  //! \param sets some IMU settings and reads some settings from IMU
  //!
  void setAndCheckIMUSettings()
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
  //!
  //! \brief createAndPublishIMUMessage
  //! \param sets some IMU settings and reads some settings from IMU
  //!
  void createAndPublishIMUMessage(std::vector<double>& parsed_val)
  {
    // orientation correction matrices in 3x3 row-major format and quaternion
    /*static const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> correction_mat(imu_orientation_correction_.data());
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

    // geometry_msgs::msg::Quaternion geometry_quaternion;
    // geometry_quaternion.x = quat.getX();
    // geometry_quaternion.y = quat.getY();
    // geometry_quaternion.z = quat.getZ();
    // geometry_quaternion.w = quat.getW();


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

    // imu_pub_->publish(imu_msg);
    // magnet_pub_->publish(magnet_msg);  // Published in teslas
    lastUpdateTime_ = this->get_clock()->now();
    last_quat_ = quat;
    msg_counter_++;
    */
  }
  //!
  //! \param buf the string to parse
  //! \param parsed_vals the previously parsed values
  //! \return number of doubles in the parsed line
  //!
  static int addToParsedVals(const std::string& buf, std::vector<double>& parsed_vals)
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

private:
  // whether or not to commit the imu settings
  bool commit_settings_;

  // whether gyroscope should be calibrated on startup
  bool calibrate_imu_;

  // IMU orientation correction.
  std::vector<double> imu_orientation_correction_;

  double orientation_rotation_;

  // frame id
  std::string frame_id_;

  // Node Handlers
  // ros::NodeHandle yostlab_priv_nh_;
  // ros::NodeHandle yostlab_nh_;
  // Node node;
  // rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_; // commenting for error

  // ros::Publisher imu_pub_;
  // imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
  // ros::Publisher magnet_pub_;
  // rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnet_pub_; // commenting for error

  // Diagnostic_updater
  std::unique_ptr<diagnostic_updater::Updater> updater;

  std::string software_version_;
  std::string calibration_mode_;
  std::string mi_mode_;
  std::string axis_direction_;
  double sensor_temp_, quaternion_length_;
  int msg_counter_;
  rclcpp::Time lastUpdateTime_;
  tf2::Quaternion last_quat_;

  // Constants
  const double GRAVITY = 9.80665;
  const double GAUSSTOTESLA = 1e-4;
  static constexpr auto MAX_IMU_TEMP = 185.0;
  static constexpr auto MIN_IMU_TEMP = -40.0;
  static constexpr auto QUATERNION_LENGTH_TOL = 0.02;
  static constexpr auto IMU_TIMEOUT_DELAY = 1.0;

  static constexpr auto SET_GYRO_ENABLED = ":107,1\n";           // enable gyroscope readings as inputs to
                                                                 // the orientation estimation
  static constexpr auto SET_ACCELEROMETER_ENABLED = ":108,1\n";  // enable accelerometer readings as inputs
                                                                 // to the orientation estimation
  static constexpr auto SET_COMPASS_ENABLED = ":109,1\n";        // enable compass readings as inputs to
                                                                 // the orientation estimation
  static constexpr auto SET_AXIS_DIRECTIONS = ":116,001\n";      // X: Right, Y: Forward, Z: Up (right-handed system)
  static constexpr auto SET_CALIB_MODE_SCALE_BIAS = ":169,1\n";  // scale bias
  static constexpr auto SET_REFERENCE_VECTOR_MODE = ":105,1\n";  // single auto continuous
  static constexpr auto SET_FILTER_MODE = ":123,1\n";            // Kalman filter mode
  static constexpr auto SET_RUNNING_AVERAGE_MODE = ":124,1\n";   // places the sensor into a confidence-based running
                                                                 // average mode, which changes the running average
                                                                 // factor based upon the confidence factor
  static constexpr auto SET_RUNNING_AVERAGE_PERCENT =
      ":117,0.45,0.45,0.40,0.45\n";  // sets what percentage of running average to use on a
                                     // component sensor

  static constexpr auto SET_OVERSAMPLE_RATE = "106:100,100,25\n";  // sets the number of times to sample each
                                                                   // component sensor for each iteration of the filter.
  /*
  Slot #1: untared orientation as quaternion [4x float]
  Slot #2: corrected gyroscope vector [3x float]
  Slot #3: corrected acceleration vector [3x float]
  Slot #4: corrected magnetometer vector [3x float]
  Slot #5: temperature in Fahrenheit [1x float]
  Slot #[6-8]: No Command
  */
  static constexpr auto SET_STREAMING_SLOTS = ":80,6,38,39,40,44,255,255,255\n";

  static constexpr auto BEGIN_GYRO_AUTO_CALIB = ":165\n";  // Performs auto-gyroscope calibration. Sensor should
                                                           // remain still while samples are taken.

  //! Orientation Commands
  static constexpr auto GET_TARED_ORIENTATION_AS_QUATERNION = ":0\n";
  static constexpr auto GET_TARED_ORIENTATION_AS_EULER_ANGLES = ":1\n";
  static constexpr auto GET_TARED_ORIENTATION_AS_ROTATION_MATRIX = ":2\n";
  static constexpr auto GET_TARED_ORIENTATION_AS_AXIS_ANGLE = ":3\n";
  static constexpr auto GET_TARED_ORIENTATION_AS_TWO_VECTOR = ":4\n";
  static constexpr auto GET_DIFFERENCE_QUATERNION = ":5\n";
  static constexpr auto GET_UNTARED_ORIENTATION_AS_QUATERNION = ":6\n";
  static constexpr auto GET_UNTARED_ORIENTATION_AS_EULER_ANGLES = ":7\n";
  static constexpr auto GET_UNTARED_ORIENTATION_AS_ROTATION_MATRIX = ":8\n";
  static constexpr auto GET_UNTARED_ORIENTATION_AS_AXIS_ANGLE = ":9\n";
  static constexpr auto GET_UNTARED_ORIENTATION_AS_TWO_VECTOR = ":10\n";
  static constexpr auto GET_TARED_TWO_VECTOR_IN_SENSOR_FRAME = ":11\n";
  static constexpr auto GET_UNTARED_TWO_VECTOR_IN_SENSOR_FRAME = ":12\n";
  //! Corrected Data Commands
  static constexpr auto GET_ALL_CORRECTED_COMPONENT_SENSOR = ":37\n";
  static constexpr auto GET_CORRECTED_GYRO_RATE = ":38\n";
  static constexpr auto GET_CORRECTED_ACCELEROMETER_VECTOR = ":39\n";
  static constexpr auto GET_CORRECTED_COMPASS_VECTOR = ":40\n";
  static constexpr auto GET_CORRECTED_LINEAR_ACCELERATION_IN_GLOBAL_SPACE = ":41\n";
  static constexpr auto CORRECT_RAW_GYRO_DATA = ":48\n";
  static constexpr auto CORRECT_RAW_ACCEL_DATA = ":49\n";
  static constexpr auto CORRECT_RAW_COMPASS_DATA = ":50\n";
  //! Other Data Commands
  static constexpr auto GET_TEMPERATURE_C = ":43\n";
  static constexpr auto GET_TEMPERATURE_F = ":44\n";
  static constexpr auto GET_CONFIDENCE_FACTOR = ":45\n";
  //! RAW Data Commands
  static constexpr auto GET_ALL_RAW_COMPONENT_SENSOR_DATA = ":64\n";
  static constexpr auto GET_RAW_GYRO_RATE = ":65\n";
  static constexpr auto GET_RAW_ACCEL_DATA = ":66\n";
  static constexpr auto GET_RAW_COMPASS_DATA = ":67\n";
  //! Streaming Commands
  static constexpr auto SET_STREAMING_SLOTS_EULER_TEMP = ":80,1,43,255,255,255,255,255,255\n";
  static constexpr auto SET_STREAMING_SLOTS_EULER_QUATERNION = ":80,1,0,255,255,255,255,255,255\n";
  static constexpr auto SET_STREAMING_SLOTS_QUATERNION_EULER = ":80,0,1,255,255,255,255,255,255\n";
  static constexpr auto SET_STREAMING_SLOTS_EULER = ":80,1,255,255,255,255,255,255,255\n";
  static constexpr auto SET_STREAMING_SLOTS_QUATERNION = ":80,0,255,255,255,255,255,255,255\n";
  static constexpr auto SET_STREAMING_SLOTS_QUATERNION_CORRECTED_GYRO_ACCELERATION_LINEAR_IN_GLOBAL = ":80,0,38,41,255,"
                                                                                                      "255,255,255,"
                                                                                                      "255\n";
  static constexpr auto SET_STREAMING_SLOTS_QUATERNION_CORRECTED_GYRO_ACCELERATION_LINEAR = ":80,0,38,39,255,255,255,"
                                                                                            "255,255\n";
  static constexpr auto GET_STREAMING_SLOTS = ":81\n";
  static constexpr auto SET_STREAMING_TIMING_5_MS = ":82,5000,0,0\n";
  static constexpr auto SET_STREAMING_TIMING_10_MS = ":82,10000,0,0\n";
  static constexpr auto SET_STREAMING_TIMING_100_MS = ":82,100000,0,0\n";
  static constexpr auto SET_STREAMING_TIMING_1000_MS = ":82,1000000,0,0\n";
  static constexpr auto SET_STREAMING_TIMING_5000_MS = ":82,5000000,0,0\n";
  static constexpr auto GET_STREAMING_TIMING = ":83\n";
  static constexpr auto GET_STREAMING_BATCH = ":84\n";
  static constexpr auto START_STREAMING = ":85\n";
  static constexpr auto STOP_STREAMING = ":86\n";
  static constexpr auto UPDATE_CURRENT_TIMESTAMP = ":95\n";
  //! Configuration Read Commands
  static constexpr auto GET_AXIS_DIRECTION = ":143\n";
  static constexpr auto GET_FILTER_MODE = ":152\n";
  static constexpr auto GET_EULER_DECOMPOSTION_ORDER = ":156\n";
  static constexpr auto GET_MI_MODE_ENABLED = ":136\n";
  //! Configuration Write Commands
  static constexpr auto SET_EULER_ANGLE_DECOMP_ORDER_XYZ = ":16,0\n";
  static constexpr auto SET_EULER_ANGLE_DECOMP_ORDER_YZX = ":16,1\n";
  static constexpr auto SET_EULER_ANGLE_DECOMP_ORDER_ZXY = ":16,2\n";
  static constexpr auto SET_EULER_ANGLE_DECOMP_ORDER_ZYX = ":16,3\n";
  static constexpr auto SET_EULER_ANGLE_DECOMP_ORDER_XZY = ":16,4\n";
  static constexpr auto SET_EULER_ANGLE_DECOMP_ORDER_YXZ = ":16,5\n";
  static constexpr auto OFFSET_WITH_CURRENT_ORIENTATION = ":19\n";
  static constexpr auto TARE_WITH_CURRENT_ORIENTATION = ":96\n";
  static constexpr auto TARE_WITH_CURRENT_QUATERNION = ":97\n";
  static constexpr auto SET_MI_MODE_ENABLED = ":112,1\n";
  static constexpr auto SET_MI_MODE_DISABLED = ":112,0\n";
  static constexpr auto SET_AXIS_DIRECTIONS_ENU = ":116,8\n";
  static constexpr auto SET_AXIS_DIRECTIONS_DEFAULT = ":116,000\n";
  static constexpr auto SET_GYRO_RANGE = ":125,1\n";
  static constexpr auto COMMIT_SETTINGS = ":225\n";
  static constexpr auto SET_FILTER_MODE_IMU = ":123,1\n";
  //! Calibration Commands
  static constexpr auto SET_CALIB_MODE_BIAS = ":169,0\n";
  static constexpr auto GET_CALIB_MODE = ":170\n";
  static constexpr auto BEGIN_MI_MODE_FIELD_CALIBRATION = ":114\n";  // Begins the calibration process for MI mode. The
                                                                     // sensor should be left in a magnetically
                                                                     // unperturbed area for 3-4 seconds after this is
                                                                     // called for calibration to succeed.
  //! System Commands
  static constexpr auto GET_FIRMWARE_VERSION_STRING = ":223\n";
  static constexpr auto RESTORE_FACTORY_SETTINGS = ":224\n";
  static constexpr auto SOFTWARE_RESET = ":226\n";
  //! logger space
  // static constexpr auto logger = "[ YostImuDriver ] ";
};  // YostLabDriver

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<YostLabDriver>();
  
  node->run();
  double spin_frequency_ = 100.0;
  rclcpp::Rate loop_rate(spin_frequency_); // Hz
  while(rclcpp::ok())
  {
    node->run2();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
