/**
Software License Agreement (BSD)

\authors   Tony Baltovski <tbaltovski@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <cstring>
#include <vector>
#include <chrono>

#include "puma_motor_driver/multi_driver_node.hpp"
#include "puma_motor_driver/driver.hpp"
#include "puma_motor_msgs/msg/multi_status.hpp"
#include "puma_motor_msgs/msg/status.hpp"
#include "puma_motor_msgs/msg/multi_feedback.hpp"
#include "puma_motor_msgs/msg/feedback.hpp"

#include "rclcpp/rclcpp.hpp"

namespace puma_motor_driver
{

MultiDriverNode::MultiDriverNode(const std::string node_name, std::vector<puma_motor_driver::Driver>& drivers)
  : Node(node_name), drivers_(drivers), active_(false)
  {
		feedback_pub_ = this->create_publisher<puma_motor_msgs::msg::MultiFeedback>("feedback", 5);
    status_pub_ = this->create_publisher<puma_motor_msgs::msg::MultiStatus>("status", 5);

    feedback_msg_.drivers_feedback.resize(drivers_.size());
    status_msg_.drivers.resize(drivers_.size());

    feedback_pub_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(40), std::bind(&MultiDriverNode::feedbackTimerCb, this));
    status_pub_timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&MultiDriverNode::statusTimerCb, this));
  }

void MultiDriverNode::publishFeedback()
{
  // Prepare output feedback message to ROS.
  uint8_t feedback_index = 0;
  for (auto& driver : drivers_)
  {
    puma_motor_msgs::msg::Feedback* f = &feedback_msg_.drivers_feedback[feedback_index];
    f->device_number = driver.deviceNumber();
    f->device_name = driver.deviceName();
    f->duty_cycle = driver.lastDutyCycle();
    f->current = driver.lastCurrent();
    f->travel = driver.lastPosition();
    f->speed = driver.lastSpeed();
    f->setpoint = driver.lastSetpoint();

    feedback_index++;
  }
  feedback_msg_.header.stamp = this->get_clock()->now();
  feedback_pub_->publish(feedback_msg_);
}

void MultiDriverNode::publishStatus()
{
  // Prepare output status message to ROS.
  uint8_t status_index = 0;
  for (auto& driver : drivers_)
  {
    puma_motor_msgs::msg::Status* s = &status_msg_.drivers[status_index];
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
  status_msg_.header.stamp = this->get_clock()->now();
  status_pub_->publish(status_msg_);
}

void MultiDriverNode::statusTimerCb()
{
  if (active_)
  {
    this->publishStatus();
  }
}

void MultiDriverNode::feedbackTimerCb()
{
  if (active_)
  {
    this->publishFeedback();
  }
}

void MultiDriverNode::activePublishers(const bool activate)
{
  active_ = activate;
}

}  // namespace puma_motor_driver
