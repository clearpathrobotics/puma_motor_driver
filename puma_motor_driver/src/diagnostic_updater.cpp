#include <string>

#include "boost/foreach.hpp"
#include "diagnostic_updater/update_functions.h"
#include "puma_motor_driver/diagnostic_updater.h"
#include "puma_motor_msgs/Status.h"

namespace puma_motor_driver
{

  typedef diagnostic_msgs::DiagnosticStatus Status;

PumaMotorDriverDiagnosticUpdater::PumaMotorDriverDiagnosticUpdater()
{
  initialized_ = false;
  setHardwareID("none");
  status_sub_ = nh_.subscribe("status", 5, &PumaMotorDriverDiagnosticUpdater::statusCallback, this);
}

const char* PumaMotorDriverDiagnosticUpdater::getModeString(uint8_t mode)
{
  switch(mode)
  {
    case puma_motor_msgs::Status::MODE_VOLTAGE:
      return "Voltage Control";
    case puma_motor_msgs::Status::MODE_CURRENT:
      return "Current Control";
    case puma_motor_msgs::Status::MODE_SPEED:
      return "Speed control";
    case puma_motor_msgs::Status::MODE_POSITION:
      return "Position control";
    case puma_motor_msgs::Status::MODE_VCOMP:
      return "Vcomp control";
    default:
      return "Unknown control";
  }
}

const char* PumaMotorDriverDiagnosticUpdater::getFaulString(uint8_t fault)
{
  switch(fault)
  {
    case puma_motor_msgs::Status::FAULT_CURRENT:
      return "current fault";
    case puma_motor_msgs::Status::FAULT_TEMPERATURE:
      return "temperature fault";
    case puma_motor_msgs::Status::FAULT_BUS_VOLTAGE:
      return "bus voltage failt";
    case puma_motor_msgs::Status::FAULT_BRIDGE_DRIVER:
      return "bridge driver fault";
    default:
      return "unknown fault";
  }
}

void PumaMotorDriverDiagnosticUpdater::driverDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat, int driver)
{
  if (last_status_->drivers[driver].fault == 0)
  {
    stat.summary(Status::OK, "motor driver is OK.");
  }
  else
  {
    stat.summaryf(Status::ERROR, "'%s' driver (%i) has a current fault.",
       getFaulString(last_status_->drivers[driver].fault));
  }

  stat.add("Driver CAN ID", static_cast<int>(last_status_->drivers[driver].device_number));
  stat.add("Driver Role", last_status_->drivers[driver].device_name.c_str());
  stat.add("Driver Mode", getModeString(last_status_->drivers[driver].mode));

  stat.add("Input terminal voltage (V)", last_status_->drivers[driver].bus_voltage);
  stat.add("Internal driver temperature (degC)", last_status_->drivers[driver].temperature);
  stat.add("Voltage as output to the motor (V)", last_status_->drivers[driver].output_voltage);
  stat.add("Value of the auxiliary ADC (V)", last_status_->drivers[driver].analog_input);
}

void PumaMotorDriverDiagnosticUpdater::statusCallback(const puma_motor_msgs::MultiStatus::ConstPtr& status_msg)
{
  last_status_ = status_msg;
  if(!initialized_)
  {
    for (int i = 0; i < status_msg->drivers.size(); i++)
    {
      char name[100];
      sprintf(name, "Puma motor driver on: %s with CAN ID (%d)",
        last_status_->drivers[i].device_name.c_str(), last_status_->drivers[i].device_number);
      add(name, boost::bind(&PumaMotorDriverDiagnosticUpdater::driverDiagnostics, this, _1, i));
    }
    initialized_ = true;
  }
  else
  {
    update();
  }

}

}  // puma_motor_driver namespace
