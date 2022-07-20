// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <vector>

#include "Gimbal2dPlugin.hpp"

#include <gazebo/common/PID.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float32.hpp>

#include <memory>

using namespace std;

geometry_msgs::msg::Quaternion RPYToQuat(double yaw, double pitch, double roll) {
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  geometry_msgs::msg::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;

  return q;
}

double QuatGetYaw(geometry_msgs::msg::Quaternion& q) {
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

double QuatGetRoll(geometry_msgs::msg::Quaternion& q) {
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    return std::atan2(sinr_cosp, cosr_cosp);
}

double QuatGetPitch(geometry_msgs::msg::Quaternion& q)  {
    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        return std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        return std::asin(sinp);
}



namespace gazebo_plugins
{
/// Class to hold private data members (PIMPL pattern)
class GimbalPluginPrivate
{
public:
  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Publisher to the gimbal status topic
  rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr pub;

  /// Subscriber to the gimbal command topic
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub;

  /// Subscriber to the pid command topic
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr pid_sub;

  /// Parent model of this plugin
  gazebo::physics::ModelPtr model;

  /// Joint for tilting the gimbal
  gazebo::physics::JointPtr tiltJoint;

  /// Joint for yawing the gimbal
  gazebo::physics::JointPtr yawJoint;

   /// Pose Stamped Message of Command Orientation
  geometry_msgs::msg::Quaternion command;

  /// Orientation of the gimbal
  geometry_msgs::msg::Quaternion orientation;

  /// PID controller for the gimbal pitch
  gazebo::common::PID pid_pitch;

  /// PID controller for the gimbal yaw
  gazebo::common::PID pid_yaw;

  /// Last update sim time
  gazebo::common::Time lastUpdateTime;

};

GimbalPlugin::GimbalPlugin()
: impl_(std::make_unique<GimbalPluginPrivate>())
{
  this->impl_->pid_pitch.Init(1, 0, 0, 0, 0, 1.0, -1.0);
  this->impl_->pid_yaw.Init(1, 0, 0, 0, 0, 1.0, -1.0);
}

GimbalPlugin::~GimbalPlugin()
{
}

void GimbalPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Create a GazeboRos node instead of a common ROS node.
  // Pass it SDF parameters so common options like namespace and remapping
  // can be handled.
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // The model pointer gives you direct access to the physics object,
  // for example:
  // RCLCPP_INFO(impl_->ros_node_->get_logger(), model->GetName().c_str());

  // Create a connection so the OnUpdate function is called at every simulation
  // iteration. Remove this call, the connection and the callback if not needed.
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GimbalPlugin::OnUpdate, this));

  // Set model
  impl_->model = model;
  
  // Get Joint Details
  std::string jointName = "tilt_joint";
  if (sdf->HasElement("pitch_joint"))
  {
    jointName = sdf->Get<std::string>("pitch_joint");
  }
  impl_->tiltJoint = impl_->model->GetJoint(jointName);
  if (!impl_->tiltJoint)
  {
    std::string scopedJointName = model->GetScopedName() + "::" + jointName;
    RCLCPP_WARN(impl_->ros_node_->get_logger(), "joint [%s] not found, trying again with scoped joint name [%s]", jointName.c_str(), scopedJointName.c_str());
    impl_->tiltJoint = impl_->model->GetJoint(scopedJointName);
  }
  if (!impl_->tiltJoint)
  {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Gimbal2dPlugin::Load ERROR! Can't get joint %s", jointName.c_str());
  }

  // Get Joint Details
  jointName = "yaw_joint";
  if (sdf->HasElement("yaw_joint"))
  {
    jointName = sdf->Get<std::string>("yaw_joint");
  }
  impl_->yawJoint = impl_->model->GetJoint(jointName);
  if (!impl_->yawJoint)
  {
    std::string scopedJointName = model->GetScopedName() + "::" + jointName;
    RCLCPP_WARN(impl_->ros_node_->get_logger(), "joint [%s] not found, trying again with scoped joint name [%s]", jointName.c_str(), scopedJointName.c_str());
    impl_->yawJoint = impl_->model->GetJoint(scopedJointName);
  }
  if (!impl_->yawJoint)
  {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Gimbal2dPlugin::Load ERROR! Can't get joint %s", jointName.c_str());
  }

  // Get initial angle details
  double p = 0;
  double y = 0;
  double r = 0;
  if (sdf->HasElement("initial_pitch"))
  {
    p = sdf->Get<double>("initial_pitch");
  }

  if (sdf->HasElement("initial_yaw"))
  {
    y = sdf->Get<double>("initial_yaw");
  }
  impl_->command = RPYToQuat(y, p, r);


  // Initialise time
  impl_->lastUpdateTime = model->GetWorld()->SimTime();

  // Gimbal state publisher
  impl_->pub = impl_->ros_node_->create_publisher<geometry_msgs::msg::Quaternion>(
    "get_gimbal_orientaiton", qos.get_publisher_qos("grasping", rclcpp::QoS(1)));
  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Advertise gimbal status on [%s]", impl_->pub->get_topic_name());

  // Gimbal subscription, callback simply sets the command
  impl_->sub = impl_->ros_node_->create_subscription<geometry_msgs::msg::Quaternion>(
    "set_gimbal_orientation", 10,
    [this](const geometry_msgs::msg::Quaternion::SharedPtr msg){
      if(msg) {
        this->impl_->command = *msg;
      } else {
        RCLCPP_WARN(this->impl_->ros_node_->get_logger(), "Received Gimbal Target Quaternion Orientation not valid");
      }
    }
  );

    // Gimbal subscription, callback simply sets the command
  impl_->pid_sub = impl_->ros_node_->create_subscription<geometry_msgs::msg::Vector3>(
    "set_gimbal_pitch_pid", 10,
    [this](const geometry_msgs::msg::Vector3::SharedPtr msg){
      this->impl_->pid_pitch.SetPGain(msg->x);
      this->impl_->pid_pitch.SetDGain(msg->y);
      this->impl_->pid_pitch.SetIGain(msg->z);
    }
  );

  impl
  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Receiving gimbal orientation quaternions on [%s]", impl_->sub->get_topic_name()); 
}

void GimbalPlugin::OnUpdate()
{
  // Do something every simulation iteration
  
  // If not initialised yet 
  if (!impl_->tiltJoint){return;}

  // Get Joint Angles
  double pitch = impl_->tiltJoint->Position(0);
  double yaw = impl_->yawJoint->Position(0);
  double roll = 0;
  impl_->orientation = RPYToQuat(yaw, pitch, roll);

  // Get current time
  gazebo::common::Time time = impl_->model->GetWorld()->SimTime();
  if (time < impl_->lastUpdateTime)
  {
    impl_->lastUpdateTime = time;
  }
  else if (time > impl_->lastUpdateTime)
  { 
    // Get Time Delta
    double dt = (impl_->lastUpdateTime - time).Double();

    // Move Pitch
    double pitch_error = pitch - QuatGetPitch(impl_->command);
    double pitch_force = impl_->pid_pitch.Update(pitch_error, dt);
    impl_->tiltJoint->SetForce(0, pitch_force);

    // Move Yaw
    double yaw_error = yaw - QuatGetYaw(impl_->command);
    double yaw_force = impl_->pid_yaw.Update(yaw_error, dt);
    impl_->yawJoint->SetForce(0, yaw_force);

    // Set Update
    impl_->lastUpdateTime = time;
  }

  impl_->pub->publish(impl_->orientation);

}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GimbalPlugin)
}  // namespace gazebo_plugins
