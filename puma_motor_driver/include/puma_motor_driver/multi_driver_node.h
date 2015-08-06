#ifndef PUMA_MOTOR_DRIVER_MULTI_DRIVER_NODE_H
#define PUMA_MOTOR_DRIVER_MULTI_DRIVER_NODE_H

#include <stdint.h>
#include <string>
#include <ros/ros.h>

#include "puma_motor_driver/driver.h"
#include "puma_motor_msgs/MultiStatus.h"
#include "puma_motor_msgs/Status.h"
#include "puma_motor_msgs/MultiFeedback.h"
#include "puma_motor_msgs/Feedback.h"

namespace puma_motor_driver
{
class MultiDriverNode
{
public:
  MultiDriverNode(ros::NodeHandle& nh, std::vector<puma_motor_driver::Driver>& drivers);

  void publishFeedback();
  void publishStatus();
  void feedbackTimerCb(const ros::TimerEvent&);
  void statusTimerCb(const ros::TimerEvent&);
private:
  ros::NodeHandle nh_;
  std::vector<puma_motor_driver::Driver>& drivers_;

  puma_motor_msgs::MultiStatus status_msg_;
  puma_motor_msgs::MultiFeedback feedback_msg_;

  ros::Publisher status_pub_;
  ros::Publisher feedback_pub_;

  ros::Timer status_pub_timer_;
  ros::Timer feedback_pub_timer_;

};

}

#endif  // PUMA_MOTOR_DRIVER_MULTI_DRIVER_NODE_H
