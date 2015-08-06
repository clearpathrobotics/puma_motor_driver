#include "puma_motor_driver/multi_driver_node.h"
#include "puma_motor_driver/driver.h"
#include "puma_motor_msgs/MultiStatus.h"
#include "puma_motor_msgs/Status.h"
#include "puma_motor_msgs/MultiFeedback.h"
#include "puma_motor_msgs/Feedback.h"
#include "boost/foreach.hpp"
#include <cstring>
#include <ros/ros.h>

namespace puma_motor_driver
{

MultiDriverNode::MultiDriverNode(ros::NodeHandle& nh, std::vector<puma_motor_driver::Driver>& drivers)
  : nh_(nh), drivers_(drivers)
  {
    feedback_pub_ = nh_.advertise<puma_motor_msgs::MultiFeedback>("feedback", 5);
    status_pub_ = nh_.advertise<puma_motor_msgs::MultiStatus>("status", 5);

    feedback_msg_.drivers_feedback.resize(drivers_.size());
    status_msg_.drivers.resize(drivers_.size());

    feedback_pub_timer_ = nh_.createTimer( ros::Duration(1.0/25), &MultiDriverNode::feedbackTimerCb, this);
    status_pub_timer_ = nh_.createTimer( ros::Duration(1.0/1), &MultiDriverNode::statusTimerCb, this);
  }

void MultiDriverNode::publishFeedback()
{
  // Prepare output feedback message to ROS.
  uint8_t feedback_index = 0;
  BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
  {
    puma_motor_msgs::Feedback* f = &feedback_msg_.drivers_feedback[feedback_index];
    f->device_number = driver.deviceNumber();
    f->device_name = driver.deviceName();
    f->duty_cycle = driver.lastDutyCycle();
    f->current = driver.lastCurrent();
    f->travel = driver.lastPosition();
    f->speed = driver.lastSpeed();
    f->setpoint = driver.lastSetpoint();

    feedback_index++;
  }
  feedback_msg_.header.stamp = ros::Time::now();
  feedback_pub_.publish(feedback_msg_);
}

void MultiDriverNode::publishStatus()
{
  // Prepare output status message to ROS.
  uint8_t status_index = 0;
  BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
  {
    puma_motor_msgs::Status* s = &status_msg_.drivers[status_index];
    s->device_number = driver.deviceNumber();
    s->device_name = driver.deviceName();
    s->bus_voltage = driver.lastBusVoltage();
    s->output_voltage = driver.lastOutVoltage();
    s->analog_input = driver.lastAnalogInput();
    s->temperature = driver.lastTemperature();
    s->mode = driver.lastMode();
    s->fault = driver.lastFault();

    status_index++;
  }
  status_msg_.header.stamp = ros::Time::now();
  status_pub_.publish(status_msg_);
}

void MultiDriverNode::statusTimerCb(const ros::TimerEvent&)
{
  publishStatus();
}

void MultiDriverNode::feedbackTimerCb(const ros::TimerEvent&)
{
  publishFeedback();
}

}  // puma_motor_driver namespace
