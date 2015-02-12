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

#include "boost/scoped_ptr.hpp"
#include "puma_motor_driver/driver.h"
#include "puma_motor_driver/serial_gateway.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "serial/serial.h"


class MultiControllerNode
{
public:

  MultiControllerNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                      puma_motor_driver::protocol::Gateway& gateway) :
    gateway_(gateway)
  {
    drivers_.push_back(puma_motor_driver::Driver(gateway_, 2, "left"));
    drivers_.push_back(puma_motor_driver::Driver(gateway_, 3, "right"));
    drivers_.push_back(puma_motor_driver::Driver(gateway_, 4, "front"));
    drivers_.push_back(puma_motor_driver::Driver(gateway_, 5, "rear"));

    cmd_sub_ = nh.subscribe("cmd", 1, &MultiControllerNode::cmd_callback, this);
  }

  void cmd_callback(const sensor_msgs::JointStateConstPtr& cmd_msg)
  {
    for (int joint = 0; joint < 4; joint++)
    {
      drivers_[joint].velocitySet(cmd_msg->velocity[joint]);
    }
  }

  void run()
  {
    ros::Rate rate(20);

    while (ros::ok())
    {
      if (!gateway_.isConnected())
      {
        if (!gateway_.connect())
        {
          ROS_ERROR("Error connecting to motor driver gateway. Retrying in 1 second.");
          ros::Duration(1.0).sleep();
          continue;
        }
        else
        {
          ROS_INFO("Connection to motor driver gateway successful.");
        }
      }

      ros::spinOnce();

      // TODO: Poll for incoming data and generate messages.


      gateway_.heartbeat();
      rate.sleep();
    }
  }

private:
  puma_motor_driver::protocol::Gateway& gateway_;
  std::vector<puma_motor_driver::Driver> drivers_;

  ros::Subscriber cmd_sub_;
};


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "puma_multi_controller_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string serial_port;
  nh_private.param<std::string>("port", serial_port, "/dev/ttyUSB0");

  serial::Serial serial;
  serial.setPort(serial_port);
  puma_motor_driver::SerialGateway gateway(serial);

  MultiControllerNode n(nh, nh_private, gateway);
  n.run();
}
