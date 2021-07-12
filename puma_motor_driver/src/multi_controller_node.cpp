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

#include <string>
#include <vector>

// #include "serial/serial.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "puma_motor_driver/driver.hpp"
// #include "puma_motor_driver/serial_gateway.h"
#include "puma_motor_driver/socketcan_gateway.hpp"
#include "puma_motor_driver/multi_driver_node.hpp"
#include "puma_motor_driver/diagnostic_updater.hpp"
#include "puma_motor_msgs/msg/multi_status.hpp"
#include "puma_motor_msgs/msg/status.hpp"
#include "puma_motor_msgs/msg/multi_feedback.hpp"
#include "puma_motor_msgs/msg/feedback.hpp"


class MultiControllerNode : public rclcpp::Node
{
public:
  MultiControllerNode(const std::string node_name) :
		Node(node_name),
    active_(false),
    status_count_(0),
    desired_mode_(puma_motor_msgs::msg::Status::MODE_SPEED)
  {
		std::string canbus_dev;
		std::unique_ptr<puma_motor_driver::Gateway> gateway;

		if (this->get_parameter("canbus_dev", canbus_dev)) {
			gateway.reset(new puma_motor_driver::SocketCANGateway(canbus_dev));
		}        
		// else if (nh_private.getParam("serial_port", serial_port))
		// {
		//     serial::Serial serial;
		//     serial.setPort(serial_port);
		//     gateway.reset(new puma_motor_driver::SerialGateway(serial));
		// }
		else {
				RCLCPP_FATAL(this->get_logger(),"No communication method given.");
				rclcpp::shutdown();
		}

		gateway_.reset(new puma_motor_driver::SocketCANGateway(canbus_dev));

    drivers_.push_back(puma_motor_driver::Driver(gateway_, 3, "fl"));
    drivers_.push_back(puma_motor_driver::Driver(gateway_, 5, "fr"));
    drivers_.push_back(puma_motor_driver::Driver(gateway_, 2, "rl"));
    drivers_.push_back(puma_motor_driver::Driver(gateway_, 4, "rr"));

		cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("cmd", 1, std::bind(&MultiControllerNode::cmdCallback, this, std::placeholders::_1));

		this->get_parameter_or("gear_ratio", gear_ratio_, 79.0);
    this->get_parameter_or("encoder_cpr", encoder_cpr_, 1024);
    this->get_parameter_or("frequency", freq_, 25);


    for (auto& driver : drivers_)
    {
      driver.clearMsgCache();
      driver.setEncoderCPR(encoder_cpr_);
      driver.setGearRatio(gear_ratio_);
      driver.setMode(desired_mode_, 0.1, 0.01, 0.0);
    }

    multi_driver_node_.reset(new puma_motor_driver::MultiDriverNode("multi_driver_node", drivers_));

		run_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000/freq_), std::bind(&MultiControllerNode::run, this));
  }

  void cmdCallback(const sensor_msgs::msg::JointState::SharedPtr cmd_msg)
  {
    if (active_)
    {
      for (auto& driver : drivers_)
      {
        for (int i = 0; i < static_cast<int>(cmd_msg->name.size()); i++)
        {
          if (driver.deviceName() == cmd_msg->name[i])
          {
            if (desired_mode_ == puma_motor_msgs::msg::Status::MODE_VOLTAGE)
            {
              driver.commandDutyCycle(cmd_msg->velocity[i]);
            }
            else if (desired_mode_ == puma_motor_msgs::msg::Status::MODE_SPEED)
            {
              driver.commandSpeed(cmd_msg->velocity[i] * 6.28);
            }
          }
        }
      }
    }
  }

  bool areAllActive()
  {
    for (auto& driver : drivers_)
    {
      if (!driver.isConfigured())
      {
        return false;
      }
    }
    return true;
  }

  bool connectIfNotConnected()
  {
    if (!gateway_->isConnected())
    {
      if (!gateway_->connect())
      {
        RCLCPP_ERROR(this->get_logger(), "Error connecting to motor driver gateway. Retrying in 1 second.");
        return false;
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Connection to motor driver gateway successful.");
      }
    }
    return true;
  }

  void run()
  {
		if (!connectIfNotConnected())
		{
			return;
		}

		if (active_)
		{
			// Checks to see if power flag has been reset for each driver
			for (auto& driver : drivers_)
			{
				if (driver.lastPower() != 0)
				{
					active_ = false;
					multi_driver_node_->activePublishers(active_);
					RCLCPP_WARN(this->get_logger(), "Power reset detected on device ID %d, will reconfigure all drivers.", driver.deviceNumber());
					for (auto& driver : drivers_)
					{
						driver.resetConfiguration();
					}
				}
			}
			// Queue data requests for the drivers in order to assemble an amalgamated status message.
			for (auto& driver : drivers_)
			{
				driver.requestStatusMessages();
				driver.requestFeedbackSetpoint();
			}
		}
		else
		{
			// Set parameters for each driver.
			for (auto& driver : drivers_)
			{
				driver.configureParams();
			}
		}

		// Process all received messages through the connected driver instances.
		puma_motor_driver::Message recv_msg;
		while (gateway_->recv(&recv_msg))
		{
			for (auto& driver : drivers_)
			{
				driver.processMessage(recv_msg);
			}
		}

		// Check parameters of each driver instance.
		if (!active_)
		{
			for (auto& driver : drivers_)
			{
				driver.verifyParams();
			}
		}

		// Verify that the all drivers are configured.
		if (areAllActive() == true && active_ == false)
		{
			active_ = true;
			multi_driver_node_->activePublishers(active_);
			RCLCPP_INFO(this->get_logger(), "All contollers active.");
		}
		// Send the broadcast heartbeat message.
		// gateway_.heartbeat();
		status_count_++;
  }

private:
  std::shared_ptr<puma_motor_driver::Gateway> gateway_;
  std::vector<puma_motor_driver::Driver> drivers_;

  int freq_;
  int encoder_cpr_;
  double gear_ratio_;
  bool active_;
  uint8_t status_count_;
  uint8_t desired_mode_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub_;
  std::shared_ptr<puma_motor_driver::MultiDriverNode> multi_driver_node_;
	rclcpp::TimerBase::SharedPtr run_timer_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exe;

  std::shared_ptr<MultiControllerNode> puma_multi_controller_node =
    std::make_shared<MultiControllerNode>("puma_multi_controller_node");

	std::shared_ptr<puma_motor_driver::PumaMotorDriverDiagnosticUpdater> puma_motor_driver_diagnostic_updater = 
		std::make_shared<puma_motor_driver::PumaMotorDriverDiagnosticUpdater>("puma_motor_driver_diagnostic_updater");

  exe.add_node(puma_multi_controller_node);
	exe.add_node(puma_motor_driver_diagnostic_updater);
  exe.spin();

  rclcpp::shutdown();
  return 0;
}
