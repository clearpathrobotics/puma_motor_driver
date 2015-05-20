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
#ifndef PUMA_MOTOR_DRIVER_DRIVER_H
#define PUMA_MOTOR_DRIVER_DRIVER_H

#include <stdint.h>
#include <string>
#include <ros/ros.h>
#include "puma_motor_driver/can_proto.h"
#include "puma_motor_msgs/Status.h"

namespace puma_motor_driver
{

class Gateway;
class Message;

class Driver
{
public:
  Driver(Gateway& gateway, uint8_t device_number, std::string device_name)
    : gateway_(gateway), device_number_(device_number), device_name_(device_name),
      configured_(false), state_(0), control_mode_(puma_motor_msgs::Status::MODE_SPEED),
      gain_p_(1), gain_i_(0), gain_d_(0), encoder_cpr_(1), gear_ratio_(1)
    {
    }

  void processMessage(const Message& received_msg);

  void sendUint8(uint32_t id, uint8_t value);
  void sendUint16(uint32_t id, uint16_t value);
  void sendFixed8x8(uint32_t id, float value);
  void sendFixed16x16(uint32_t id, float value);


  /**
   * Sends messages to the motor controller requesting all missing elements to
   * populate the cache of status data. Returns true if any messages were sent,
   * false if the cache is already complete.
   */
  bool requestStatusMessages();

  /**
   * Sends messages to the motor controller requesting all missing elements to
   * populate the cache of status data. Returns true if any messages were sent,
   * false if the cache is already complete.
   */
  bool requestFeedbackMessages();

  /**
   * Clear the received flags from the status cache, in preparation for the next
   * request batch to go out.
   */
  void clearStatusCache();

  /**
   * Switch to open-loop voltage control, and command the supplied value.
   *
   * @param[in] cmd Value to command, ranging from -1.0 to 1.0, where zero is neutral.
   */
  void commandDutyCycle(float cmd);

  void commandSpeed(float cmd);
  //void currentSet(float cmd);
  //void positionSet(float cmd);
  //void neutralSet();

  void setEncoderCPR(uint16_t encoder_cpr);
  void setGearRatio(float gear_ratio);
  void setMode(uint8_t mode);
  void setMode(uint8_t mode, float p, float i, float d);
  void setGains(float p, float i, float d);

  float lastDutyCycle();
  float lastBusVoltage();
  float lastCurrent();
  float lastTemperature();
  float lastPosition();
  float lastSpeed();
  uint8_t lastFault();
  uint8_t lastPower();
  float lastOutVoltage();
  uint8_t lastMode();

  void configureParams();
  void verifyParams();
  bool isConfigured();
  void resetConfiguration();

  uint8_t posEncoderRef();
  uint8_t spdEncoderRef();
  uint16_t encoderCounts();

  float getP();
  float getI();
  float getD();

  /**  **CURRENTLY NOT USED**
   * Return the current duty cycle of the motor driver's h-bridge from the status cache.
   */
  float statusDutyCycleGet();
  float statusSpeedGet();

  /** Assignment operator, necessary on GCC 4.8 to copy instances
   *  into a vector. */
  Driver operator=(const Driver& rhs)
  {
    return Driver(gateway_, device_number_, device_name_);
  }

  std::string deviceName() { return device_name_; }

  uint8_t deviceNumber() { return device_number_; }

private:
  Gateway& gateway_;
  uint8_t device_number_;
  std::string device_name_;

  bool configured_;
  uint8_t state_;

  uint8_t control_mode_;
  float gain_p_;
  float gain_i_;
  float gain_d_;
  uint16_t encoder_cpr_;
  float gear_ratio_;

  struct StatusField
  {
    uint8_t data[4];
    bool received;

    float interpretFixed8x8()
    {
      return static_cast<int8_t>(data[1]) + static_cast<float>(data[0]) / 256.0f;
    }

    double interpretFixed16x16()
    {
      return ((data[0] | static_cast<int32_t>(data[1]) << 8 | static_cast<int32_t>(data[2]) << 16 | static_cast<int32_t>(data[3]) << 24)) / double(1<<16);
    }
  };
  StatusField status_fields_[11];

  StatusField* statusFieldForMessage(const Message& msg);
};

}

#endif  // PUMA_MOTOR_DRIVER_DRIVER_H
