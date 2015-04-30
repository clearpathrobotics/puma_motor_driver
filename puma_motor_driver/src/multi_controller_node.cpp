/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
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

#include "boost/foreach.hpp"
#include "boost/scoped_ptr.hpp"
#include "puma_motor_driver/driver.h"
#include "puma_motor_driver/serial_gateway.h"
#include "puma_motor_driver/socketcan_gateway.h"
#include "puma_motor_msgs/MultiStatus.h"
#include "puma_motor_msgs/Status.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "serial/serial.h"


class MultiControllerNode
{
public:

  MultiControllerNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                      puma_motor_driver::Gateway& gateway) :
    gateway_(gateway)
  {
    drivers_.push_back(puma_motor_driver::Driver(gateway_, 3, "fl"));
    drivers_.push_back(puma_motor_driver::Driver(gateway_, 5, "fr"));
    drivers_.push_back(puma_motor_driver::Driver(gateway_, 2, "rl"));
    drivers_.push_back(puma_motor_driver::Driver(gateway_, 4, "rr"));

    cmd_sub_ = nh.subscribe("cmd", 1, &MultiControllerNode::cmdCallback, this);
    status_pub_ = nh.advertise<puma_motor_msgs::MultiStatus>("status", 5);

    status_msg_.drivers.resize(4);
  }

  void cmdCallback(const sensor_msgs::JointStateConstPtr& cmd_msg)
  {
    // TODO: Match joint names rather than assuming indexes align.
    for (int joint = 0; joint < 4; joint++)
    {
      drivers_[joint].commandDutyCycle(cmd_msg->velocity[joint]);
    }
  }

  bool connectIfNotConnected()
  {
    if (!gateway_.isConnected())
    {
      if (!gateway_.connect())
      {
        ROS_ERROR("Error connecting to motor driver gateway. Retrying in 1 second.");
        return false;
      }
      else
      {
        ROS_INFO("Connection to motor driver gateway successful.");
      }
    }
    return true;
  }

  void run()
  {
    ros::Rate rate(25);

    while (ros::ok())
    {
      if (!connectIfNotConnected())
      {
        ros::Duration(1.0).sleep();
        continue;
      }

      // Process ROS callbacks, which will queue command messages to the drivers.
      ros::spinOnce();
      gateway_.sendAllQueued();
      ros::Duration(0.005).sleep();

      // Queue data requests for the drivers in order to assemble an amalgamated status message.
      BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
      {
        driver.clearStatusCache();
        driver.requestStatusMessages();
        gateway_.sendAllQueued();
        ros::Duration(0.006).sleep();
      }

      // Send all queued messages.

      // Process all received messages through the connected driver instances.
      puma_motor_driver::Message recv_msg;
      while (gateway_.recv(&recv_msg))
      {
        BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
        {
          driver.processMessage(recv_msg);
        }
      }

      // Prepare output status message to ROS.
      uint8_t status_index = 0;
      BOOST_FOREACH(puma_motor_driver::Driver& driver, drivers_)
      {
        puma_motor_msgs::Status* s = &status_msg_.drivers[status_index];
        s->device_number = driver.deviceNumber();
        s->device_name = driver.deviceName();
        s->duty_cycle = driver.lastDutyCycle();
        s->bus_voltage = driver.lastBusVoltage();
        s->current = driver.lastCurrent();
        s->travel = driver.lastPosition();
        s->fault = driver.lastFault();
        s->mode = driver.lastMode();
        s->output_voltage = driver.lastOutVoltage();

        status_index++;
      }
      status_pub_.publish(status_msg_);

      // Send the broadcast heartbeat message.
      // gateway_.heartbeat();
      rate.sleep();
    }
  }

private:
  puma_motor_driver::Gateway& gateway_;
  std::vector<puma_motor_driver::Driver> drivers_;

  ros::Subscriber cmd_sub_;

  // Maintain a persistent MultiStatus to avoid doing vector resizes on
  // every trip through the main loop.
  puma_motor_msgs::MultiStatus status_msg_;
  ros::Publisher status_pub_;
};


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "puma_multi_controller_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string serial_port;
  std::string canbus_dev;

  boost::scoped_ptr<puma_motor_driver::Gateway> gateway;

  if (nh_private.getParam("canbus_dev", canbus_dev))
  {
    gateway.reset(new puma_motor_driver::SocketCANGateway (canbus_dev));
  }
  else if (nh_private.getParam("serial_port", serial_port))
  {
    serial::Serial serial;
    serial.setPort(serial_port);
    gateway.reset(new puma_motor_driver::SerialGateway (serial));
  }
  else
  {
    ROS_FATAL("No communication method given.");
    return 1;
  }

  MultiControllerNode n(nh, nh_private, *gateway);
  n.run();

}
