#ifndef PUMA_MOTOR_DRIVER_DIAGNOSTIC_UPDATER_H
#define PUMA_MOTOR_DRIVER_DIAGNOSTIC_UPDATER_H

#include <string>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/publisher.h"
#include "puma_motor_msgs/MultiStatus.h"
#include "puma_motor_msgs/Status.h"

namespace puma_motor_driver
{

class PumaMotorDriverDiagnosticUpdater : private diagnostic_updater::Updater
{
public:
  PumaMotorDriverDiagnosticUpdater();

  void driverDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat, int driver);

  void statusCallback(const puma_motor_msgs::MultiStatus::ConstPtr& status_msg);

private:
  ros::NodeHandle nh_;
  ros::Subscriber status_sub_;
  puma_motor_msgs::MultiStatus::ConstPtr last_status_;
  bool initialized_;

  static const char* getFaulString(uint8_t fault);
  static const char* getModeString(uint8_t mode);
};

}  // puma_motor_driver namespace

#endif  // PUMA_MOTOR_DRIVER_DIAGNOSTIC_UPDATER_H
