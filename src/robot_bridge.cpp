#include "robot_bridge/robot_bridge.h"

using namespace robot_bridge;

RobotBridge::RobotBridge() : nh_(""), nh_local_("~") {
  nh_local_.param<double>("loop_rate", p_loop_rate_, 100.0);

  nh_local_.param<std::string>("port_name", p_port_name_, std::string("/dev/ttyS0"));
  nh_local_.param<int>("baud_rate", p_baud_rate_, 115200);
  nh_local_.param<int>("flow_control", p_flow_control_, 0);
  nh_local_.param<int>("parity", p_parity_, 0);
  nh_local_.param<int>("stop_bits", p_stop_bits_, 1);
  nh_local_.param<int>("char_size", p_char_size_, 8);

  nh_local_.param<std::string>("map_frame", p_map_frame_id_, std::string("map"));
  nh_local_.param<std::string>("odom_frame", p_odom_frame_id_, std::string("odom"));
  nh_local_.param<std::string>("robot_frame", p_robot_frame_id_, std::string("robot"));

  serial_port_.open(p_port_name_);
  serial_port_.setBaudRate(p_baud_rate_);
  serial_port_.setFlowControl(p_flow_control_);
  serial_port_.setCharSize(p_char_size_);
  serial_port_.setParity(p_parity_);
  serial_port_.setStopBits(p_stop_bits_);

  timer_ = nh_.createTimer(ros::Duration(1.0 / p_loop_rate_), &RobotBridge::timerCallback, this, false, false);
  timer_.setPeriod(ros::Duration(1.0 / p_loop_rate_), false);
  timer_.start();
}

RobotBridge::~RobotBridge() {
  serial_port_.close();
}

void RobotBridge::timerCallback(const ros::TimerEvent& e) {
  serial_port_.write("AAA", 3);
}

void RobotBridge::publishOdom(const Message& msg) {
  ros::Time now = ros::Time::now();
  geometry_msgs::Quaternion rotation = tf::createQuaternionMsgFromYaw(0.0);

  geometry_msgs::TransformStamped odom_tf;

  odom_tf.header.stamp = now;
  odom_tf.header.frame_id = p_odom_frame_id_;
  odom_tf.child_frame_id = p_robot_frame_id_;

  odom_tf.transform.translation.x = 0.0;
  odom_tf.transform.translation.y = 0.0;
  odom_tf.transform.translation.z = 0.0;
  odom_tf.transform.rotation = rotation;

  tf_bc_.sendTransform(odom_tf);

  nav_msgs::OdometryPtr odom_msg(new nav_msgs::Odometry);

  odom_msg->header.stamp = now;
  odom_msg->header.frame_id = p_odom_frame_id_;
  odom_msg->child_frame_id = p_robot_frame_id_;

  odom_msg->pose.pose.position.x = 0.0;
  odom_msg->pose.pose.position.y = 0.0;
  odom_msg->pose.pose.position.z = 0.0;
  odom_msg->pose.pose.orientation = rotation;

  odom_msg->twist.twist.linear.x = 0.0;
  odom_msg->twist.twist.linear.y = 0.0;
  odom_msg->twist.twist.angular.z = 0.0;

  odom_pub_.publish(odom_msg);
}
