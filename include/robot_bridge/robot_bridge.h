#pragma once

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include "serial_port.h"
#include "message.h"

namespace robot_bridge
{

class RobotBridge {
public:
  RobotBridge();
  ~RobotBridge();

private:
  void timerCallback(const ros::TimerEvent& e);
  void publishOdom(const Message& msg);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::Timer timer_;

  ros::Publisher odom_pub_;

  tf::TransformBroadcaster tf_bc_;
  tf::TransformListener tf_ls_;

  SerialPort serial_port_;

  // Parameters
  double p_loop_rate_;

  std::string p_port_name_;
  int p_baud_rate_;
  int p_flow_control_;
  int p_parity_;
  int p_stop_bits_;
  int p_char_size_;

  std::string p_robot_frame_id_;
  std::string p_odom_frame_id_;
  std::string p_map_frame_id_;
};

} // namespace robot_bridge

