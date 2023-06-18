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

// ROS
#include "rclcpp/rclcpp.hpp"

// Messages.
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

// Original
// #include "ypspur_ros/compatibility.h"
#include "ypspur_ros/msg/joint_position_control.hpp"

// Google
#include <glog/logging.h>

// STL
#include <map>
#include <string>

static const std::string NODE_NAME = "joint_position_to_joint_trajectory";
static const std::string SUBSCRIBE_TOPIC_JOINT_POSITION = "joint_position";
static const std::string SUBSCRIBE_TOPIC_JOINT_STATE = "joint_states";
static const std::string PUBLISH_TOPIC_JOINT_TRAJECTORY = "joint_trajectory";

static const int DEFAULT_QUEUE_SIZE = 5;

class ConvertNode : public rclcpp::Node {
 private:
  struct Params {
    double accel_;
    bool skip_same_;
  };

 private:
  // X. State
  Params params_;
  std::map<std::string, double> state_;

  // X. Publishers
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      p_pub_joint_traj_;

  // X. Subscribers
  rclcpp::Subscription<ypspur_ros::msg::JointPositionControl>::SharedPtr
      p_sub_joint_pos_ctrl_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      p_sub_joint_state_;

 private:
  void cbJointState(const sensor_msgs::msg::JointState::ConstSharedPtr& msg)
  {
    for (size_t i = 0; i < msg->name.size(); i++)
    {
      state_[msg->name[i]] = msg->position[i];
    }
  }
  ypspur_ros::msg::JointPositionControl cmd_prev;
  void cbJointPosition(const ypspur_ros::msg::JointPositionControl::ConstSharedPtr& msg)
  {
    while (true)
    {
      if (msg->joint_names.size() != cmd_prev.joint_names.size())
        break;
      {
        bool eq = true;
        for (unsigned int i = 0; i < msg->joint_names.size(); i++)
        {
          if (msg->joint_names[i].compare(cmd_prev.joint_names[i]) != 0)
            eq = false;
        }
        if (!eq)
          break;
      }
      if (msg->positions.size() != cmd_prev.positions.size()) 
        break;
      {
        bool eq = true;
        for (unsigned int i = 0; i < msg->positions.size(); i++)
        {
          if (msg->positions[i] != cmd_prev.positions[i])
            eq = false;
        }
        if (!eq)
          break;
      }
      if (msg->velocities.size() != cmd_prev.velocities.size())
        break;
      {
        bool eq = true;
        for (unsigned int i = 0; i < msg->velocities.size(); i++)
        {
          if (msg->velocities[i] != cmd_prev.velocities[i])
            eq = false;
        }
        if (!eq)
          break;
      }
      if (msg->accelerations.size() != cmd_prev.accelerations.size())
        break;
      {
        bool eq = true;
        for (unsigned int i = 0; i < msg->accelerations.size(); i++)
        {
          if (msg->accelerations[i] != cmd_prev.accelerations[i])
            eq = false;
        }
        if (!eq)
          break;
      }
      return;
    }
    cmd_prev = *msg;

    trajectory_msgs::msg::JointTrajectory cmd;
    cmd.header = msg->header;
    cmd.joint_names = msg->joint_names;
    cmd.points.resize(1);
    cmd.points[0].velocities.resize(msg->positions.size());
    cmd.points[0].positions = msg->positions;
    cmd.points[0].accelerations.resize(msg->positions.size());

    float t_max = 0;
    int i = 0;
    for (auto& p : msg->positions)
    {
      float t = fabs(p - state_[msg->joint_names[i]]) / msg->velocities[i];
      if (t_max < t)
        t_max = t;

      i++;
    }
    cmd.points[0].time_from_start = rclcpp::Duration::from_seconds(t_max);

    p_pub_joint_traj_->publish(cmd);
  }

 public:
  ConvertNode(const rclcpp::NodeOptions& options)
      : Node(NODE_NAME,
             rclcpp::NodeOptions(options)
                 .allow_undeclared_parameters(true)
                 .automatically_declare_parameters_from_overrides(true)) {
    CHECK(this->get_parameter("accel", params_.accel_));
    CHECK(this->get_parameter("skip_same", params_.skip_same_));

    // X. Register publishers.
    p_pub_joint_traj_ =
        this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            PUBLISH_TOPIC_JOINT_TRAJECTORY,
            rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QUEUE_SIZE)));

    // X. Register subscribers.
    p_sub_joint_state_ =
        this->create_subscription<sensor_msgs::msg::JointState>(
            SUBSCRIBE_TOPIC_JOINT_STATE,
            rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QUEUE_SIZE)),
            std::bind(&ConvertNode::cbJointState, this, std::placeholders::_1));
    p_sub_joint_pos_ctrl_ =
        this->create_subscription<ypspur_ros::msg::JointPositionControl>(
            SUBSCRIBE_TOPIC_JOINT_POSITION,
            rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QUEUE_SIZE)),
            std::bind(&ConvertNode::cbJointPosition, this,
                      std::placeholders::_1));
  }
};

int main(int argc, char* argv[]) 
{
  rclcpp::init(argc, argv);

  ConvertNode conv(rclcpp::NodeOptions{});
  rclcpp::spin(conv.shared_from_this());

  return 0;
}

// clang-format on