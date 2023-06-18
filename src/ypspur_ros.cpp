// clang-format off
/*
 * Copyright (c) 2015-2017, the ypspur_ros authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Message (Original)
#include "ypspur_ros/msg/control_mode.hpp"
#include "ypspur_ros/msg/digital_input.hpp"
#include "ypspur_ros/msg/digital_output.hpp"
#include "ypspur_ros/msg/joint_position_control.hpp"

// Message (ROS)
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"
#include "tf2/time.h"

// Google
#include <glog/logging.h>

// System
#include <signal.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/types.h>
#include <sys/wait.h>

// STL
#include <exception>
#include <map>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>

// #include <compatibility.h>

namespace YP
{
#include <ypspur.h>
}  // namespace YP

bool g_shutdown = false;
void sigintHandler(int)
{
  g_shutdown = true;
}

template <typename T>
T read_parameter(rclcpp::Node &node, const std::string &param_name, 
  const T& default_val) {
  node.declare_parameter(param_name, default_val);

  T value;
  CHECK(node.get_parameter(param_name, value));
  return value;
}

class YpspurRosNode : public rclcpp::Node 
{
private:
  rclcpp::executors::SingleThreadedExecutor executor_;

  // X. Publishers
  rclcpp::Publisher<ypspur_ros::msg::DigitalInput>::SharedPtr p_pub_digital_input_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr p_pub_wrench_stamped_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr p_pub_odometry_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr p_pub_joint_state_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr p_pubs_diag_array_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> p_pubs_ad_values_;

  // X. Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr p_sub_cmd_vel_;
  rclcpp::Subscription<ypspur_ros::msg::JointPositionControl>::SharedPtr p_sub_joint_pos_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr p_sub_joint_traj_;
  rclcpp::Subscription<ypspur_ros::msg::ControlMode>::SharedPtr p_sub_ctrl_mode_;
  std::map<std::string, rclcpp::Subscription<ypspur_ros::msg::DigitalOutput>::SharedPtr> p_subs_digital_outputs_;
  std::map<std::string, rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> p_subs_float32_values_;

  // Transform related.
  const tf2::Vector3 z_axis_;
  std::unique_ptr<tf2_ros::Buffer> p_tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> p_tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> p_tf_broadcaster_;

  std::string port_;
  std::string param_file_;
  std::string ypspur_bin_;
  std::map<std::string, std::string> frames_;
  std::map<std::string, double> params_;
  int key_;
  bool simulate_;
  bool simulate_control_;

  double tf_time_offset_;

  pid_t pid_;

  enum OdometryMode
  {
    DIFF,
    NONE
  };
  OdometryMode mode_;
  class JointParams
  {
  public:
    int id_;
    std::string name_;
    double accel_;
    double vel_;
    double angle_ref_;
    double vel_ref_;
    double vel_end_;
    enum control_mode_
    {
      STOP,
      VELOCITY,
      POSITION,
      TRAJECTORY
    };
    control_mode_ control_;
    trajectory_msgs::msg::JointTrajectory cmd_joint_;
  };
  std::vector<JointParams> joints_;
  std::map<std::string, int> joint_name_to_num_;

  class AdParams
  {
  public:
    bool enable_;
    std::string name_;
    double gain_;
    double offset_;
  };
  class DioParams
  {
  public:
    bool enable_;
    std::string name_;
    bool input_;
    bool output_;
  };
  bool digital_input_enable_;
  std::vector<AdParams> ads_;
  std::vector<DioParams> dios_;
  static const int AD_NUM_ = 8;
  unsigned int dio_output_;
  unsigned int dio_dir_;
  unsigned int dio_output_default_;
  unsigned int dio_dir_default_;

  static const int DIO_NUM_ = 8;
  std::map<int, rclcpp::Time> dio_revert_;

  int device_error_state_;
  int device_error_state_prev_;
  rclcpp::Time device_error_state_time_;

  geometry_msgs::msg::Twist::ConstSharedPtr cmd_vel_;
  rclcpp::Time cmd_vel_time_;
  rclcpp::Duration cmd_vel_expire_;

  int control_mode_;

  bool avoid_publishing_duplicated_odom_;
  bool publish_odom_tf_;
  rclcpp::Time previous_odom_stamp_;

  void cbControlMode(const ypspur_ros::msg::ControlMode::ConstSharedPtr& msg)
  {
    control_mode_ = msg->vehicle_control_mode;
    switch (control_mode_)
    {
      case ypspur_ros::msg::ControlMode::OPEN:
        YP::YP_openfree();
        break;
      case ypspur_ros::msg::ControlMode::TORQUE:
        YP::YPSpur_free();
        break;
      case ypspur_ros::msg::ControlMode::VELOCITY:
        break;
    }
  }
  void cbCmdVel(const geometry_msgs::msg::Twist::ConstSharedPtr& msg)
  {
    cmd_vel_ = msg;
    cmd_vel_time_ = rclcpp::Clock(rcl_clock_type_t::RCL_ROS_TIME).now();
    if (control_mode_ == ypspur_ros::msg::ControlMode::VELOCITY)
    {
      YP::YPSpur_vel(msg->linear.x, msg->angular.z);
    }
  }
  void cbJoint(const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr& msg)
  {
    const rclcpp::Time now = rclcpp::Clock(rcl_clock_type_t::RCL_ROS_TIME).now();

    std_msgs::msg::Header header = msg->header;
    if (rclcpp::Time(header.stamp) == rclcpp::Time(static_cast<int64_t>(0), rcl_clock_type_t::RCL_ROS_TIME))
      header.stamp = now;

    std::map<std::string, trajectory_msgs::msg::JointTrajectory> new_cmd_joints;
    size_t i = 0;
    for (const std::string& name : msg->joint_names)
    {
      trajectory_msgs::msg::JointTrajectory cmd_joint;
      cmd_joint.header = header;
      cmd_joint.joint_names.resize(1);
      cmd_joint.joint_names[0] = name;
      cmd_joint.points.clear();
      std::string err_msg;
      for (auto& cmd : msg->points)
      {
        if (rclcpp::Time(header.stamp) + cmd.time_from_start < now)
        {
          RCLCPP_ERROR(this->get_logger(),
                "Ignored outdated JointTrajectory command "
                "(joint: %s, now: %0.6lf, stamp: %0.6lf, time_from_start: %0.6lf)",
                name.c_str(), now.seconds(), rclcpp::Time(header.stamp).seconds(), 
                rclcpp::Duration(cmd.time_from_start).seconds());
          break;
        }

        trajectory_msgs::msg::JointTrajectoryPoint p;
        p.time_from_start = cmd.time_from_start;
        p.positions.resize(1);
        p.velocities.resize(1);
        if (cmd.velocities.size() <= i)
          p.velocities[0] = 0.0;
        else
          p.velocities[0] = cmd.velocities[i];
        p.positions[0] = cmd.positions[i];

        cmd_joint.points.push_back(p);
      }
      i++;

      if (cmd_joint.points.size() != msg->points.size())
        return;

      new_cmd_joints[name] = cmd_joint;
    }
    // Apply if all JointTrajectoryPoints are valid
    for (auto& new_cmd_joint : new_cmd_joints)
    {
      const int joint_num = joint_name_to_num_[new_cmd_joint.first];
      joints_[joint_num].control_ = JointParams::TRAJECTORY;
      joints_[joint_num].cmd_joint_ = new_cmd_joint.second;
    }
  }
  void cbSetVel(const std_msgs::msg::Float32::ConstSharedPtr& msg, int num)
  {
    // printf("set_vel %d %d %f\n", num, joints_[num].id_, msg->data);
    joints_[num].vel_ = msg->data;
    YP::YP_set_joint_vel(joints_[num].id_, joints_[num].vel_);
  }
  void cbSetAccel(const std_msgs::msg::Float32::ConstSharedPtr& msg, int num)
  {
    // printf("set_accel %d %d %f\n", num, joints_[num].id_, msg->data);
    joints_[num].accel_ = msg->data;
    YP::YP_set_joint_accel(joints_[num].id_, joints_[num].accel_);
  }
  void cbVel(const std_msgs::msg::Float32::ConstSharedPtr& msg, int num)
  {
    // printf("vel_ %d %d %f\n", num, joints_[num].id_, msg->data);
    joints_[num].vel_ref_ = msg->data;
    joints_[num].control_ = JointParams::VELOCITY;
    YP::YP_joint_vel(joints_[num].id_, joints_[num].vel_ref_);
  }
  void cbAngle(const std_msgs::msg::Float32::ConstSharedPtr& msg, int num)
  {
    joints_[num].angle_ref_ = msg->data;
    joints_[num].control_ = JointParams::POSITION;
    YP::YP_joint_ang(joints_[num].id_, joints_[num].angle_ref_);
  }
  void cbJointPosition(const ypspur_ros::msg::JointPositionControl::ConstSharedPtr& msg)
  {
    int i = 0;
    for (auto& name : msg->joint_names)
    {
      if (joint_name_to_num_.find(name) == joint_name_to_num_.end())
      {
        RCLCPP_ERROR(this->get_logger(), "Unknown joint name '%s'", name.c_str());
        continue;
      }
      int num = joint_name_to_num_[name];
      // printf("%s %d %d  %f", name.c_str(), num, joints_[num].id_, msg->positions[i]);
      joints_[num].vel_ = msg->velocities[i];
      joints_[num].accel_ = msg->accelerations[i];
      joints_[num].angle_ref_ = msg->positions[i];
      joints_[num].control_ = JointParams::POSITION;

      YP::YP_set_joint_vel(joints_[num].id_, joints_[num].vel_);
      YP::YP_set_joint_accel(joints_[num].id_, joints_[num].accel_);
      YP::YP_joint_ang(joints_[num].id_, joints_[num].angle_ref_);
      i++;
    }
  }

  void cbDigitalOutput(const ypspur_ros::msg::DigitalOutput::ConstSharedPtr& msg, int id_)
  {
    const auto dio_output_prev = dio_output_;
    const auto dio_dir_prev = dio_dir_;
    const unsigned int mask = 1 << id_;

    switch (msg->output)
    {
      case ypspur_ros::msg::DigitalOutput::HIGH_IMPEDANCE:
        dio_output_ &= ~mask;
        dio_dir_ &= ~mask;
        break;
      case ypspur_ros::msg::DigitalOutput::LOW:
        dio_output_ &= ~mask;
        dio_dir_ |= mask;
        break;
      case ypspur_ros::msg::DigitalOutput::HIGH:
        dio_output_ |= mask;
        dio_dir_ |= mask;
        break;
      case ypspur_ros::msg::DigitalOutput::PULL_UP:
        dio_output_ |= mask;
        dio_dir_ &= ~mask;
        break;
      case ypspur_ros::msg::DigitalOutput::PULL_DOWN:
        RCLCPP_ERROR(this->get_logger(), "Digital IO pull down is not supported on this system");
        break;
    }
    if (dio_output_ != dio_output_prev)
      YP::YP_set_io_data(dio_output_);
    if (dio_dir_ != dio_dir_prev)
      YP::YP_set_io_dir(dio_dir_);

    if (rclcpp::Duration(msg->toggle_time) > rclcpp::Duration::from_seconds(0.0))
    {
      dio_revert_[id_] = rclcpp::Clock(rcl_clock_type_t::RCL_ROS_TIME).now() + msg->toggle_time;
    }
  }
  void revertDigitalOutput(int id_)
  {
    const auto dio_output_prev = dio_output_;
    const auto dio_dir_prev = dio_dir_;
    const unsigned int mask = 1 << id_;

    dio_output_ &= ~mask;
    dio_output_ |= dio_output_default_ & mask;
    dio_dir_ &= ~mask;
    dio_dir_ |= dio_output_default_ & mask;

    if (dio_output_ != dio_output_prev)
      YP::YP_set_io_data(dio_output_);
    if (dio_dir_ != dio_dir_prev)
      YP::YP_set_io_dir(dio_dir_);

    dio_revert_[id_] = rclcpp::Time(static_cast<int64_t>(0), rcl_clock_type_t::RCL_ROS_TIME);
  }
  void updateDiagnostics(const rclcpp::Time& now, const bool connection_down = false)
  {
    const int connection_error = connection_down ? 1 : YP::YP_get_error_state();
    double t = 0;

    int err = 0;
    if (!connection_error)
      t = YP::YP_get_device_error_state(0, &err);
    device_error_state_ |= err;

    if (device_error_state_time_ + rclcpp::Duration::from_seconds(1.0) < now || connection_down ||
        device_error_state_ != device_error_state_prev_)
    {
      device_error_state_time_ = now;
      device_error_state_prev_ = device_error_state_;

      diagnostic_msgs::msg::DiagnosticArray msg;
      msg.header.stamp = now;
      msg.status.resize(1);
      msg.status[0].name = "YP-Spur Motor Controller";
      msg.status[0].hardware_id = "ipc-key" + std::to_string(key_);
      if (device_error_state_ == 0 && connection_error == 0)
      {
        if (t == 0)
        {
          msg.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::OK;
          msg.status[0].message = "Motor controller doesn't "
                                  "provide device error state.";
        }
        else
        {
          if (rclcpp::Time(static_cast<int64_t>(t * 1e9), rcl_clock_type_t::RCL_ROS_TIME) < now - rclcpp::Duration::from_seconds(1.0))
          {
            msg.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            msg.status[0].message = "Motor controller doesn't "
                                    "update latest device error state.";
          }
          else
          {
            msg.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            msg.status[0].message = "Motor controller is running without error.";
          }
        }
      }
      else
      {
        msg.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        if (connection_error)
          msg.status[0].message +=
              "Connection to ypspur-coordinator is down.";
        if (device_error_state_)
          msg.status[0].message +=
              std::string((msg.status[0].message.size() > 0 ? " " : "")) +
              "Motor controller reported error id " +
              std::to_string(device_error_state_) + ".";
      }
      msg.status[0].values.resize(2);
      msg.status[0].values[0].key = "connection_error";
      msg.status[0].values[0].value = std::to_string(connection_error);
      msg.status[0].values[1].key = "device_error";
      msg.status[0].values[1].value = std::to_string(device_error_state_);

      p_pubs_diag_array_->publish(msg);
      device_error_state_ = 0;
    }
  }

public:
  YpspurRosNode(const rclcpp::NodeOptions& options) 
    : Node("ypspur_ros_node", 
           rclcpp::NodeOptions(options)
               .allow_undeclared_parameters(true)
               .automatically_declare_parameters_from_overrides(true))
    , executor_()
    , p_pub_digital_input_()
    , p_pub_wrench_stamped_()
    , p_pub_odometry_()
    , p_pub_joint_state_()
    , p_pubs_diag_array_()
    , p_pubs_ad_values_()
    , p_sub_cmd_vel_()
    , p_sub_joint_pos_()
    , p_sub_joint_traj_()
    , p_sub_ctrl_mode_()
    , p_subs_digital_outputs_()
    , p_subs_float32_values_()
    , z_axis_(0, 0, 1)
    , p_tf_buffer_()
    , p_tf_listener_()
    , p_tf_broadcaster_()
    , port_{}
    , param_file_{}
    , ypspur_bin_{}
    , frames_{}
    , params_{}
    , key_{}
    , simulate_{}
    , simulate_control_{}
    , tf_time_offset_{}
    , pid_{}
    , mode_{}
    , joints_{}
    , joint_name_to_num_{}
    , digital_input_enable_{}
    , ads_{}
    , dios_{}
    , dio_output_{}
    , dio_dir_{}
    , dio_output_default_{}
    , dio_dir_default_{}
    , dio_revert_{}
    , device_error_state_(0)
    , device_error_state_prev_(0)
    , device_error_state_time_(0)
    , cmd_vel_{}
    , cmd_vel_time_{}
    , cmd_vel_expire_(0, 0)
    , control_mode_{}
    , avoid_publishing_duplicated_odom_(true)
    , publish_odom_tf_(true)
    , previous_odom_stamp_{}
  {
    // X. Initialize Tf.
    p_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    p_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*p_tf_buffer_);
    p_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    //compat::checkCompatMode();

    // X. Read parameters.
    port_ = read_parameter<std::string>(*this, "port", std::string("/dev/ttyACM0"));
    key_ = read_parameter<int>(*this, "ipc_key", 28741);
    simulate_ = read_parameter<bool>(*this, "simulate", false);
    simulate_control_ = read_parameter<bool>(*this, "simulate_control", false);
    if (simulate_control_) {
      simulate_ = true;      
    }
    ypspur_bin_ = read_parameter<std::string>(*this, "ypspur_bin", std::string("ypspur-coordinator"));
    param_file_ = read_parameter<std::string>(*this, "param_file", std::string(""));
    tf_time_offset_ = read_parameter<double>(*this, "tf_time_offset", 0.0);
    {
      double cmd_vel_expire_s = read_parameter<double>(*this, "cmd_vel_expire", -1.0);
      cmd_vel_expire_ = rclcpp::Duration::from_seconds(cmd_vel_expire_s);
    }

    // X. Read parameters for ad.
    std::string ad_mask("");
    ads_.resize(AD_NUM_);
    for (int i = 0; i < AD_NUM_; i++)
    {
      ads_[i].enable_ = read_parameter<bool>(
        *this, std::string("ad") + std::to_string(i) + std::string("_enable"),
          false);
      ads_[i].name_ = read_parameter<std::string>(
        *this, std::string("ad") + std::to_string(i) + std::string("_name"),
        std::string("ad") + std::to_string(i));
      ads_[i].gain_ = read_parameter<double>(
        *this, std::string("ad") + std::to_string(i) + std::string("_gain"),
        1.0);
      ads_[i].offset_ = read_parameter<double>(
        *this, std::string("ad") + std::to_string(i) + std::string("_offset"),
        0.0);

      ad_mask = (ads_[i].enable_ ? std::string("1") : std::string("0")) + ad_mask;
      p_pubs_ad_values_.insert(
        std::make_pair("ad/" + ads_[i].name_,
        this->create_publisher<std_msgs::msg::Float32>("ad/" + ads_[i].name_, 1)));
    }

    // X. Read parameters for diode.
    digital_input_enable_ = false;
    dio_output_default_ = 0;
    dio_dir_default_ = 0;
    dios_.resize(DIO_NUM_);
    for (int i = 0; i < DIO_NUM_; i++)
    {
      DioParams param;
      param.enable_ = read_parameter<bool>(
        *this, std::string("dio") + std::to_string(i) + std::string("_enable"), false);
      if (param.enable_) 
      {
        param.name_ = read_parameter<std::string>(
          *this, std::string("dio") + std::to_string(i) + std::string("_name"),
          std::string("dio") + std::to_string(i));
        param.output_ = read_parameter<bool>(
          *this, std::string("dio") + std::to_string(i) + std::string("_output"),
          true);
        param.input_ = read_parameter<bool>(
          *this, std::string("dio") + std::to_string(i) + std::string("_input"),
          false);

        if (param.output_)
        {
          std::function<void(const ypspur_ros::msg::DigitalOutput::ConstSharedPtr &)> callback 
            = std::bind(&YpspurRosNode::cbDigitalOutput, this, std::placeholders::_1, i);
          p_subs_digital_outputs_.insert(std::make_pair(
            param.name_, this->create_subscription<ypspur_ros::msg::DigitalOutput>(
              param.name_, 1, callback)));
        }

        std::string output_default = read_parameter<std::string>(
          *this, std::string("dio") + std::to_string(i) + std::string("_default"),
          std::string("HIGH_IMPEDANCE"));
        if (output_default.compare("HIGH_IMPEDANCE") == 0)
        {
        }
        else if (output_default.compare("LOW") == 0)
        {
          dio_dir_default_ |= 1 << i;
        }
        else if (output_default.compare("HIGH") == 0)
        {
          dio_dir_default_ |= 1 << i;
          dio_output_default_ |= 1 << i;
        }
        else if (output_default.compare("PULL_UP") == 0)
        {
          dio_output_default_ |= 1 << i;
        }
        else if (output_default.compare("PULL_DOWN") == 0)
        {
          RCLCPP_ERROR(this->get_logger(), "Digital IO pull down is not supported on this system");
        }
        if (param.input_)
          digital_input_enable_ = true;
      }
      dios_[i] = param;
    }
    dio_output_ = dio_output_default_;
    dio_dir_ = dio_dir_default_;
    if (digital_input_enable_)
    {
      p_pub_digital_input_ = 
        this->create_publisher<ypspur_ros::msg::DigitalInput>("digital_input",
          rclcpp::QoS(rclcpp::KeepLast(2)));
    }

    frames_["odom"] = read_parameter<std::string>(*this, "odom_id", std::string("odom"));
    frames_["base_link"] = read_parameter<std::string>(*this, "base_link_id", std::string("base_link"));
    frames_["origin"] = read_parameter<std::string>(*this, "origin_id", std::string(""));
    params_["hz"] = read_parameter<double>(*this, "hz", 200.0);

    std::string mode_name = read_parameter<std::string>(*this, "OdometryMode", std::string("diff"));
    if (mode_name.compare("diff") == 0)
    {
      mode_ = DIFF;
      p_pub_wrench_stamped_ = 
        this->create_publisher<geometry_msgs::msg::WrenchStamped>(
          "wrench", rclcpp::QoS(rclcpp::KeepLast(1)));
      p_pub_odometry_ = 
        this->create_publisher<nav_msgs::msg::Odometry>(
          "odom", rclcpp::QoS(rclcpp::KeepLast(1)));
      p_sub_cmd_vel_ = 
        this->create_subscription<geometry_msgs::msg::Twist>(
          "cmd_vel", rclcpp::QoS(rclcpp::KeepLast(1)), 
            std::bind(&YpspurRosNode::cbCmdVel, this, std::placeholders::_1));
      avoid_publishing_duplicated_odom_ = read_parameter<bool>(
        *this, "avoid_publishing_duplicated_odom", true);
      publish_odom_tf_ = read_parameter<bool>(
        *this, "publish_odom_tf", true);
    }
    else if (mode_name.compare("none") == 0)
    {
    }
    else
    {
      throw(std::runtime_error("unknown mode: " + mode_name));
    }

    int max_joint_id = read_parameter<int>(*this, "max_joint_id", 32);
    bool separated_joint = read_parameter<bool>(*this, "separated_joint_control", false);
    int num = 0;
    for (int i = 0; i < max_joint_id; i++)
    {
      std::string name;
      name = std::string("joint") + std::to_string(i);
      if (this->get_parameter(name + std::string("_enable")).get_type() 
          != rclcpp::ParameterType::PARAMETER_NOT_SET)
      {
        bool en = read_parameter<bool>(
            *this, name + std::string("_enable"), false);
        if (en)
        {
          JointParams jp;
          jp.id_ = i;
          jp.name_ = read_parameter<std::string>(*this, name + std::string("_name"), name);
          jp.accel_ = read_parameter<double>(*this, name + std::string("_accel"), 3.14);
          joint_name_to_num_[jp.name_] = num;
          joints_.push_back(jp);
          // printf("%s %d %d", jp.name_.c_str(), jp.id_, joint_name_to_num_[jp.name_]);
          if (separated_joint)
          {
            {
              std::function<void(const std_msgs::msg::Float32::ConstSharedPtr&)> callback = 
                std::bind(&YpspurRosNode::cbSetVel, this, std::placeholders::_1, num);
              p_subs_float32_values_.insert(
                std::make_pair(jp.name_ + std::string("_setVel"),
                this->create_subscription<std_msgs::msg::Float32>(
                  jp.name_ + std::string("/set_vel"), rclcpp::QoS(rclcpp::KeepLast(1)), callback)));
            }
            {
              std::function<void(const std_msgs::msg::Float32::ConstSharedPtr&)> callback = 
                std::bind(&YpspurRosNode::cbSetAccel, this, std::placeholders::_1, num);
              p_subs_float32_values_.insert(
                std::make_pair(jp.name_ + std::string("_setAccel"),
                this->create_subscription<std_msgs::msg::Float32>(
                  jp.name_ + std::string("/set_accel"), rclcpp::QoS(rclcpp::KeepLast(1)), callback)));
            }
            {
              std::function<void(const std_msgs::msg::Float32::ConstSharedPtr&)> callback = 
                std::bind(&YpspurRosNode::cbVel, this, std::placeholders::_1, num);
              p_subs_float32_values_.insert(
                std::make_pair(jp.name_ + std::string("_vel"),
                this->create_subscription<std_msgs::msg::Float32>(
                  jp.name_ + std::string("/vel"), rclcpp::QoS(rclcpp::KeepLast(1)), callback)));
            }
            {
              std::function<void(const std_msgs::msg::Float32::ConstSharedPtr&)> callback = 
                std::bind(&YpspurRosNode::cbAngle, this, std::placeholders::_1, num);
              p_subs_float32_values_.insert(
                std::make_pair(jp.name_ + std::string("_pos"),
                this->create_subscription<std_msgs::msg::Float32>(
                  jp.name_ + std::string("/pos"), rclcpp::QoS(rclcpp::KeepLast(1)), callback)));
            }
          }
          p_sub_joint_pos_ = 
            this->create_subscription<ypspur_ros::msg::JointPositionControl>(
              std::string("joint_position"), rclcpp::QoS(rclcpp::KeepLast(1)), 
              std::bind(&YpspurRosNode::cbJointPosition, this, std::placeholders::_1));
          num++;
        }
      }
    }
    if (joints_.size() > 0)
    {
      p_pub_joint_state_ = 
        this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(rclcpp::KeepLast(2)));
      p_sub_joint_traj_ = 
        this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
          "joint_trajectory", rclcpp::QoS(rclcpp::KeepLast(joints_.size() * 2)), 
          std::bind(&YpspurRosNode::cbJoint, this, std::placeholders::_1));
    }
    p_sub_ctrl_mode_ = this->create_subscription<ypspur_ros::msg::ControlMode>(
          "control_mode", rclcpp::QoS(rclcpp::KeepLast(1)), 
          std::bind(&YpspurRosNode::cbControlMode, this, std::placeholders::_1));
    control_mode_ = ypspur_ros::msg::ControlMode::VELOCITY;

    p_pubs_diag_array_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(rclcpp::KeepLast(1)));

    pid_ = 0;
    for (int i = 0; i < 2; i++)
    {
      if (i > 0 || YP::YPSpur_initex(key_) < 0)
      {
        std::vector<std::string> args =
            {
              ypspur_bin_,
              "-d", port_,
              "--admask", ad_mask,
              "--msq-key", std::to_string(key_)
            };
        if (digital_input_enable_)
          args.push_back(std::string("--enable-get-digital-io"));
        if (simulate_)
          args.push_back(std::string("--without-device"));
        if (param_file_.size() > 0)
        {
          args.push_back(std::string("-p"));
          args.push_back(param_file_);
        }

        char** argv = new char*[args.size() + 1];
        for (unsigned int i = 0; i < args.size(); i++)
        {
          argv[i] = new char[args[i].size() + 1];
          memcpy(argv[i], args[i].c_str(), args[i].size());
          argv[i][args[i].size()] = 0;
        }
        argv[args.size()] = nullptr;

        int msq = msgget(key_, 0666 | IPC_CREAT);
        msgctl(msq, IPC_RMID, nullptr);

        RCLCPP_WARN(this->get_logger(), "launching ypspur-coordinator");
        pid_ = fork();
        if (pid_ == -1)
        {
          const int err = errno;
          throw(std::runtime_error(std::string("failed to fork process: ") + strerror(err)));
        }
        else if (pid_ == 0)
        {
          execvp(ypspur_bin_.c_str(), argv);
          throw(std::runtime_error("failed to start ypspur-coordinator"));
        }

        for (unsigned int i = 0; i < args.size(); i++)
          delete argv[i];
        delete argv;

        for (int i = 4; i >= 0; i--)
        {
          int status;
          if (waitpid(pid_, &status, WNOHANG) == pid_)
          {
            if (WIFSIGNALED(status))
            {
              throw(std::runtime_error(
                  "ypspur-coordinator dead immediately by signal " + std::to_string(WTERMSIG(status))));
            }
            if (WIFEXITED(status))
            {
              throw(std::runtime_error(
                  "ypspur-coordinator dead immediately with exit code " + std::to_string(WEXITSTATUS(status))));
            }
            throw(std::runtime_error("ypspur-coordinator dead immediately"));
          }
          else if (i == 0)
          {
            throw(std::runtime_error("failed to init libypspur"));
          }
          rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(1*1e9)));
          if (YP::YPSpur_initex(key_) >= 0)
            break;
        }
      }
      double ret;
      std::atomic<bool> done(false);
      auto get_vel_thread = [&ret, &done]
      {
        double test_v, test_w;
        ret = YP::YPSpur_get_vel(&test_v, &test_w);
        done = true;
      };
      std::thread spur_test = std::thread(get_vel_thread);
      rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(0.1 * 1e9)));
      if (!done)
      {
        // There is no way to kill thread safely in C++11
        // So, just leave it detached.
        spur_test.detach();
        RCLCPP_WARN(this->get_logger(), "ypspur-coordinator seems to be down - launching");
        continue;
      }
      spur_test.join();
      if (ret < 0)
      {
        RCLCPP_WARN(this->get_logger(), "ypspur-coordinator returns error - launching");
        continue;
      }
      RCLCPP_WARN(this->get_logger(), "ypspur-coordinator launched");
      break;
    }

    RCLCPP_INFO(this->get_logger(), "ypspur-coordinator conneceted");
    signal(SIGINT, sigintHandler);

    YP::YP_get_parameter(YP::YP_PARAM_MAX_VEL, &params_["vel"]);
    YP::YP_get_parameter(YP::YP_PARAM_MAX_ACC_V, &params_["acc"]);
    YP::YP_get_parameter(YP::YP_PARAM_MAX_W, &params_["angvel"]);
    YP::YP_get_parameter(YP::YP_PARAM_MAX_ACC_W, &params_["angacc"]);
    
    if (this->get_parameter("vel").get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
      RCLCPP_WARN(this->get_logger(), "default \"vel\" %0.3f used", (float)params_["vel"]);
    if (this->get_parameter("acc").get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
      RCLCPP_WARN(this->get_logger(), "default \"acc\" %0.3f used", (float)params_["acc"]);
    if (this->get_parameter("angvel").get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
      RCLCPP_WARN(this->get_logger(), "default \"angvel\" %0.3f used", (float)params_["angvel"]);
    if (this->get_parameter("angacc").get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
      RCLCPP_WARN(this->get_logger(), "default \"angacc\" %0.3f used", (float)params_["angacc"]);

    params_["vel"] = read_parameter<double>(*this, "vel", params_["vel"]);
    params_["acc"] = read_parameter<double>(*this, "acc", params_["acc"]);
    params_["angvel"] = read_parameter<double>(*this, "angvel", params_["angvel"]);
    params_["angacc"] = read_parameter<double>(*this, "angacc", params_["angacc"]);

    YP::YPSpur_set_vel(params_["vel"]);
    YP::YPSpur_set_accel(params_["acc"]);
    YP::YPSpur_set_angvel(params_["angvel"]);
    YP::YPSpur_set_angaccel(params_["angacc"]);

    YP::YP_set_io_data(dio_output_);
    YP::YP_set_io_dir(dio_dir_);

    executor_.add_node(this->shared_from_this());
  }
  ~YpspurRosNode()
  {
    // Kill ypspur-coordinator if the communication is still active.
    if (pid_ > 0 && YP::YP_get_error_state() == 0)
    {
      RCLCPP_INFO(this->get_logger(), "killing ypspur-coordinator (%d)", (int)pid_);
      kill(pid_, SIGINT);
      int status;
      waitpid(pid_, &status, 0);
      RCLCPP_INFO(this->get_logger(), "ypspur-coordinator is killed (status: %d)", status);
    }
  }
  bool spin()
  {
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.frame_id = frames_["odom"];
    odom_trans.child_frame_id = frames_["base_link"];

    nav_msgs::msg::Odometry odom;
    geometry_msgs::msg::WrenchStamped wrench;
    odom.header.frame_id = frames_["odom"];
    odom.child_frame_id = frames_["base_link"];
    wrench.header.frame_id = frames_["base_link"];

    odom.pose.pose.position.x = 0;
    odom.pose.pose.position.y = 0;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(z_axis_, 0));
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = 0;

    std::map<int, geometry_msgs::msg::TransformStamped> joint_trans;
    sensor_msgs::msg::JointState joint;
    if (joints_.size() > 0)
    {
      joint.header.frame_id = std::string("");
      joint.velocity.resize(joints_.size());
      joint.position.resize(joints_.size());
      joint.effort.resize(joints_.size());
      for (auto& j : joints_)
        joint.name.push_back(j.name_);

      for (unsigned int i = 0; i < joints_.size(); i++)
      {
        joint_trans[i].header.frame_id = joints_[i].name_ + std::string("_in");
        joint_trans[i].child_frame_id = joints_[i].name_ + std::string("_out");
        joint.velocity[i] = 0;
        joint.position[i] = 0;
        joint.effort[i] = 0;
      }
    }

    RCLCPP_INFO(this->get_logger(), "ypspur_ros main loop started");
    rclcpp::Rate loop(params_["hz"]);
    while (!g_shutdown)
    {
      const rclcpp::Time now = this->get_clock()->now();
      const float dt = 1.0 / params_["hz"];

      if (cmd_vel_ && cmd_vel_expire_ > rclcpp::Duration::from_seconds(0.0))
      {
        if (cmd_vel_time_ + cmd_vel_expire_ < now)
        {
          // cmd_vel is too old and expired
          cmd_vel_ = nullptr;
          if (control_mode_ == ypspur_ros::msg::ControlMode::VELOCITY)
            YP::YPSpur_vel(0.0, 0.0);
        }
      }

      if (mode_ == DIFF)
      {
        double x, y, yaw, v(0), w(0);
        double t;

        if (!simulate_control_)
        {
          t = YP::YPSpur_get_pos(YP::CS_BS, &x, &y, &yaw);
          if (t <= 0.0)
            break;
          YP::YPSpur_get_vel(&v, &w);
        }
        else
        {
          t = this->get_clock()->now().seconds();
          if (cmd_vel_)
          {
            v = cmd_vel_->linear.x;
            w = cmd_vel_->angular.z;
          }
          yaw = tf2::getYaw(odom.pose.pose.orientation) + dt * w;
          x = odom.pose.pose.position.x + dt * v * cosf(yaw);
          y = odom.pose.pose.position.y + dt * v * sinf(yaw);
        }

        const rclcpp::Time current_stamp(static_cast<int64_t>(t * 1e9), rcl_clock_type_t::RCL_ROS_TIME);
        if (!avoid_publishing_duplicated_odom_ || (current_stamp > previous_odom_stamp_))
        {
          odom.header.stamp = current_stamp;
          odom.pose.pose.position.x = x;
          odom.pose.pose.position.y = y;
          odom.pose.pose.position.z = 0;
          odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(z_axis_, yaw));
          odom.twist.twist.linear.x = v;
          odom.twist.twist.linear.y = 0;
          odom.twist.twist.angular.z = w;
          p_pub_odometry_->publish(odom);

          if (publish_odom_tf_)
          {
            odom_trans.header.stamp = current_stamp + rclcpp::Duration::from_seconds(tf_time_offset_);
            odom_trans.transform.translation.x = x;
            odom_trans.transform.translation.y = y;
            odom_trans.transform.translation.z = 0;
            odom_trans.transform.rotation = odom.pose.pose.orientation;
            p_tf_broadcaster_->sendTransform(odom_trans);
          }
        }
        previous_odom_stamp_ = current_stamp;

        if (!simulate_control_)
        {
          t = YP::YPSpur_get_force(&wrench.wrench.force.x, &wrench.wrench.torque.z);
          if (t <= 0.0)
            break;
        }
        wrench.header.stamp = rclcpp::Time(static_cast<int64_t>(t * 1e9), rcl_clock_type_t::RCL_ROS_TIME);;
        wrench.wrench.force.y = 0;
        wrench.wrench.force.z = 0;
        wrench.wrench.torque.x = 0;
        wrench.wrench.torque.y = 0;
        p_pub_wrench_stamped_->publish(wrench);

        if (frames_["origin"].length() > 0)
        {
          try
          {
            tf2::Stamped<tf2::Transform> transform;
            geometry_msgs::msg::TransformStamped transform_msg = 
              p_tf_buffer_->lookupTransform(
                frames_["origin"], frames_["base_link"], 
                  rclcpp::Time(static_cast<int64_t>(0), rcl_clock_type_t::RCL_ROS_TIME));
            tf2::fromMsg(transform_msg, transform);

            tf2Scalar yaw, pitch, roll;
            transform.getBasis().getEulerYPR(yaw, pitch, roll);
            YP::YPSpur_adjust_pos(YP::CS_GL, transform.getOrigin().x(),
                                  transform.getOrigin().y(),
                                  yaw);
          }
          catch (tf2::TransformException& ex)
          {
            RCLCPP_ERROR(this->get_logger(), 
              "Failed to feedback localization result to YP-Spur (%s)", ex.what());
          }
        }
      }
      if (joints_.size() > 0)
      {
        double t;
        if (!simulate_control_)
        {
          t = -1.0;
          while (t < 0.0)
          {
            int i = 0;
            for (auto& j : joints_)
            {
              const double t0 = YP::YP_get_joint_ang(j.id_, &joint.position[i]);
              const double t1 = YP::YP_get_joint_vel(j.id_, &joint.velocity[i]);
              const double t2 = YP::YP_get_joint_torque(j.id_, &joint.effort[i]);

              if (t0 != t1 || t1 != t2)
              {
                // Retry if updated during this joint
                t = -1.0;
                break;
              }
              if (t < 0.0)
              {
                t = t0;
              }
              else if (t != t0)
              {
                // Retry if updated during loops
                t = -1.0;
                break;
              }
              i++;
            }
          }
          if (t <= 0.0)
            break;
          joint.header.stamp = rclcpp::Time(static_cast<int64_t>(t * 1e9), rcl_clock_type_t::RCL_ROS_TIME);
        }
        else
        {
          t = this->get_clock()->now().seconds();
          for (unsigned int i = 0; i < joints_.size(); i++)
          {
            auto vel_prev = joint.velocity[i];
            switch (joints_[i].control_)
            {
              case JointParams::STOP:
                break;
              case JointParams::TRAJECTORY:
              case JointParams::POSITION:
              case JointParams::VELOCITY:
                switch (joints_[i].control_)
                {
                  case JointParams::POSITION:
                  {
                    float position_err = joints_[i].angle_ref_ - joint.position[i];
                    joints_[i].vel_ref_ = sqrtf(2.0 * joints_[i].accel_ * fabs(position_err));
                    if (joints_[i].vel_ref_ > joints_[i].vel_)
                      joints_[i].vel_ref_ = joints_[i].vel_;
                    if (position_err < 0)
                      joints_[i].vel_ref_ = -joints_[i].vel_ref_;
                  }
                  break;
                  case JointParams::TRAJECTORY:
                  {
                    float position_err = joints_[i].angle_ref_ - joint.position[i];
                    float v_sq = joints_[i].vel_end_ * joints_[i].vel_end_ + 2.0 * joints_[i].accel_ * position_err;
                    joints_[i].vel_ref_ = sqrtf(fabs(v_sq));

                    float vel_max;
                    if (fabs(joints_[i].vel_) < fabs(joints_[i].vel_end_))
                    {
                      if (fabs(position_err) <
                          (joints_[i].vel_end_ * joints_[i].vel_end_ - joints_[i].vel_ * joints_[i].vel_) /
                              (2.0 * joints_[i].accel_))
                        vel_max = fabs(joints_[i].vel_end_);
                      else
                        vel_max = joints_[i].vel_;
                    }
                    else
                      vel_max = joints_[i].vel_;

                    if (joints_[i].vel_ref_ > vel_max)
                      joints_[i].vel_ref_ = vel_max;
                    if (position_err < 0)
                      joints_[i].vel_ref_ = -joints_[i].vel_ref_;
                  }
                  break;
                  default:
                    break;
                }
                joint.velocity[i] = joints_[i].vel_ref_;
                if (joint.velocity[i] < vel_prev - dt * joints_[i].accel_)
                {
                  joint.velocity[i] = vel_prev - dt * joints_[i].accel_;
                }
                else if (joint.velocity[i] > vel_prev + dt * joints_[i].accel_)
                {
                  joint.velocity[i] = vel_prev + dt * joints_[i].accel_;
                }
                joint.position[i] += joint.velocity[i] * dt;
                break;
            }
          }
          joint.header.stamp = rclcpp::Time(static_cast<int64_t>(t * 1e9), rcl_clock_type_t::RCL_ROS_TIME);
        }
        p_pub_joint_state_->publish(joint);

        for (unsigned int i = 0; i < joints_.size(); i++)
        {
          joint_trans[i].transform.rotation = tf2::toMsg(tf2::Quaternion(z_axis_, joint.position[i]));
          joint_trans[i].header.stamp = \
            rclcpp::Time(static_cast<int64_t>(t * 1e9), 
              rcl_clock_type_t::RCL_ROS_TIME) + rclcpp::Duration::from_seconds(tf_time_offset_);
          p_tf_broadcaster_->sendTransform(joint_trans[i]);
        }

        for (unsigned int jid = 0; jid < joints_.size(); jid++)
        {
          if (joints_[jid].control_ != JointParams::TRAJECTORY)
            continue;

          auto& cmd_joint_ = joints_[jid].cmd_joint_;
          auto t = now - cmd_joint_.header.stamp;
          if (t < rclcpp::Duration::from_seconds(0.0))
            continue;

          bool done = true;
          for (auto& cmd : cmd_joint_.points)
          {
            if (rclcpp::Duration(cmd.time_from_start) < rclcpp::Duration::from_seconds(0.0))
              continue;
            if (now > rclcpp::Time(cmd_joint_.header.stamp) + cmd.time_from_start)
              continue;
            done = false;

            double ang_err = cmd.positions[0] - joint.position[jid];
            double& vel_end_ = cmd.velocities[0];
            double& vel_start = joint.velocity[jid];
            auto t_left = rclcpp::Duration(cmd.time_from_start) - t;

            double v;
            double v_min;
            bool v_found = true;
            while (true)
            {
              // RCLCPP_INFO(this->get_logger(), 
              //   "st: %0.3f, en: %0.3f, err: %0.3f, t: %0.3f", vel_start, vel_end_, ang_err, t_left.seconds());
              int s;
              if (vel_end_ > vel_start)
                s = 1;
              else
                s = -1;
              v = (s * (vel_start + vel_end_) * (vel_start - vel_end_) +
                   ang_err * joints_[jid].accel_ * 2.0) /
                  (2.0 * s * (vel_start - vel_end_) + joints_[jid].accel_ * 2.0 * (t_left.seconds()));

              double err_deacc;
              err_deacc = fabs(vel_end_ * vel_end_ - v * v) / (joints_[jid].accel_ * 2.0);
              // RCLCPP_INFO(this->get_logger(), "v+-: %0.3f", v);
              v_min = fabs(v);
              if ((vel_start * s <= v * s || err_deacc >= fabs(ang_err)) &&
                  v * s <= vel_end_ * s)
                break;

              v_min = DBL_MAX;

              auto vf = [](const double& st, const double& en,
                           const double& acc, const double& err, const double& t,
                           const int& sig, const int& sol, double& ret)
              {
                double sq;
                sq = -4.0 * st * st +
                     8.0 * st * en -
                     4.0 * en * en +
                     4.0 * sig * acc * 2 * (t * (st + en) - 2.0 * err) +
                     4.0 * acc * acc * t * t;
                if (sq < 0)
                  return false;

                ret = (2.0 * sig * (st + en) + 2.0 * acc * t + sol * sqrt(sq)) / (4.0 * sig);

                return true;
              };

              v_found = false;

              if (vf(vel_start, vel_end_, joints_[jid].accel_, ang_err, t_left.seconds(),
                     1, 1, v))
              {
                // RCLCPP_INFO(this->get_logger(), "v++: sol+ %0.3f", v);
                if (v >= vel_start && v >= vel_end_)
                {
                  if (v_min > fabs(v))
                    v_min = fabs(v);
                  v_found = true;
                }
              }
              if (vf(vel_start, vel_end_, joints_[jid].accel_, ang_err, t_left.seconds(),
                     -1, 1, v))
              {
                // RCLCPP_INFO(this->get_logger(), "v--: sol+ %0.3f", v);
                if (v <= vel_start && v <= vel_end_)
                {
                  if (v_min > fabs(v))
                    v_min = fabs(v);
                  v_found = true;
                }
              }
              if (vf(vel_start, vel_end_, joints_[jid].accel_, ang_err, t_left.seconds(),
                     1, -1, v))
              {
                // RCLCPP_INFO(this->get_logger(), "v++: sol- %0.3f", v);
                if (v >= vel_start && v >= vel_end_)
                {
                  if (v_min > fabs(v))
                    v_min = fabs(v);
                  v_found = true;
                }
              }
              if (vf(vel_start, vel_end_, joints_[jid].accel_, ang_err, t_left.seconds(),
                     -1, -1, v))
              {
                // RCLCPP_INFO(this->get_logger(), "v--: sol- %0.3f", v);
                if (v <= vel_start && v <= vel_end_)
                {
                  if (v_min > fabs(v))
                    v_min = fabs(v);
                  v_found = true;
                }
              }
              break;
            }
            if (v_found)
            {
              // RCLCPP_INFO(this->get_logger(), "v: %0.3f", v_min);
              joints_[jid].angle_ref_ = cmd.positions[0];
              joints_[jid].vel_end_ = vel_end_;
              joints_[jid].vel_ = v_min;

              YP::YP_set_joint_vel(joints_[jid].id_, v_min);
              YP::YP_set_joint_accel(joints_[jid].id_, joints_[jid].accel_);
              YP::YP_joint_ang_vel(joints_[jid].id_, cmd.positions[0], vel_end_);
            }
            else
            {
              RCLCPP_ERROR(this->get_logger(), "Impossible trajectory given");
            }
            break;
          }

          if (done)
          {
            if ((joints_[jid].vel_end_ > 0.0 &&
                 joints_[jid].angle_ref_ > joint.position[jid] &&
                 joints_[jid].angle_ref_ < joint.position[jid] + joints_[jid].vel_ref_ * dt) ||
                (joints_[jid].vel_end_ < 0.0 &&
                 joints_[jid].angle_ref_ < joint.position[jid] &&
                 joints_[jid].angle_ref_ > joint.position[jid] + joints_[jid].vel_ref_ * dt))
            {
              joints_[jid].control_ = JointParams::VELOCITY;
              joints_[jid].vel_ref_ = joints_[jid].vel_end_;
            }
          }
        }
      }

      for (int i = 0; i < AD_NUM_; i++)
      {
        if (ads_[i].enable_)
        {
          std_msgs::msg::Float32 ad;
          ad.data = YP::YP_get_ad_value(i) * ads_[i].gain_ + ads_[i].offset_;
          p_pubs_ad_values_.at("ad/" + ads_[i].name_)->publish(ad);
        }
      }

      if (digital_input_enable_)
      {
        ypspur_ros::msg::DigitalInput din;

        din.header.stamp = this->get_clock()->now();
        int in = YP::YP_get_ad_value(15);
        for (int i = 0; i < DIO_NUM_; i++)
        {
          if (!dios_[i].enable_)
            continue;
          din.name.push_back(dios_[i].name_);
          if (in & (1 << i))
            din.state.push_back(true);
          else
            din.state.push_back(false);
        }
        p_pub_digital_input_->publish(din);
      }

      for (int i = 0; i < DIO_NUM_; i++)
      {
        if (dio_revert_[i] != rclcpp::Time(static_cast<int64_t>(0), rcl_clock_type_t::RCL_ROS_TIME))
        {
          if (dio_revert_[i] < now)
          {
            revertDigitalOutput(i);
          }
        }
      }
      updateDiagnostics(now);

      if (YP::YP_get_error_state())
        break;

      executor_.spin_once();
      loop.sleep();

      int status;
      if (waitpid(pid_, &status, WNOHANG) == pid_)
      {
        if (WIFEXITED(status))
        {
          RCLCPP_ERROR(this->get_logger(), "ypspur-coordinator exited");
        }
        else
        {
          if (WIFSTOPPED(status))
          {
            RCLCPP_ERROR(this->get_logger(), "ypspur-coordinator dead with signal %d", 
              WSTOPSIG(status));
          }
          else
          {
            RCLCPP_ERROR(this->get_logger(), "ypspur-coordinator dead");
          }
          updateDiagnostics(now, true);
        }
        break;
      }
    }
    RCLCPP_INFO(this->get_logger(), "ypspur_ros main loop terminated");

    if (YP::YP_get_error_state())
    {
      RCLCPP_ERROR(this->get_logger(), "ypspur-coordinator is not active");
      return false;
    }
    return true;
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);


  int ret = 0;

  try
  {
    YpspurRosNode yr(rclcpp::NodeOptions{});
    if (!yr.spin())
      ret = 1;
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("global_logger"), "%s", e.what());
    ret = 1;
  }

  rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(0.1*1e9)));
  return ret;
}

// clang-format on