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

// Message (ROS)
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// ROS
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

// System
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>

// STL
#include <map>
#include <string>

class JointTfPublisherNode : public rclcpp::Node {
 private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr p_sub_jnt_state_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> p_tf_broadcaster_;
  const tf2::Vector3 z_axis_;

  void cbJoint(const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
    for (size_t i = 0; i < msg->name.size(); i++)
    {
      geometry_msgs::msg::TransformStamped trans;
      trans.header = msg->header;
      trans.header.frame_id = msg->name[i] + "_in";
      trans.child_frame_id = msg->name[i] + "_out";

      trans.transform.rotation = tf2::toMsg(tf2::Quaternion(z_axis_, msg->position[i]));
      p_tf_broadcaster_->sendTransform(trans);
    }
  }

 public:
  JointTfPublisherNode(const rclcpp::NodeOptions &options)
    : Node("joint_tf_publisher", 
           rclcpp::NodeOptions(options)
               .allow_undeclared_parameters(true)
               .automatically_declare_parameters_from_overrides(true))
    , p_tf_broadcaster_{}
    , z_axis_(0, 0, 1)
  {
    p_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // X. Subscriber.
    p_sub_jnt_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::QoS(rclcpp::KeepLast(1)), 
      std::bind(&JointTfPublisherNode::cbJoint, this, std::placeholders::_1));
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  JointTfPublisherNode jp(rclcpp::NodeOptions{});
  rclcpp::spin(jp.shared_from_this());

  return 0;
}
// clang-format on