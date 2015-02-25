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

#include "puma_motor_driver/driver.h"
#include "puma_motor_driver/gateway.h"
#include "puma_motor_driver/message.h"

#include <cstring>
#include <ros/ros.h>

namespace puma_motor_driver
{

void Driver::processMessage(const Message& received_msg)
{
  // If it's not our message, jump out.
  if (received_msg.getDeviceNumber() != device_number_) return;

  // If there's no data then this is a request message, jump out.
  if (received_msg.len == 0) return;

  StatusField* field = statusFieldForMessage(received_msg);

  // Wasn't a status message, return.
  if (!field) return;

  // Copy the received data and mark that field as received.
  memcpy(field->data, received_msg.data, received_msg.len);
  field->received = true;
}

void Driver::commandDutyCycle(float cmd)
{
  Message driver_msg;
  driver_msg.id = LM_API_VOLT_SET | device_number_;
  driver_msg.len = 2;

  int16_t output_vel = 32767 * cmd;
  memcpy(driver_msg.data, &output_vel, 2);

  gateway_.queue(driver_msg);
}

void Driver::clearStatusCache()
{
  // Set it all to zero, which will in part clear
  // the boolean flags to be false.
  memset(status_fields_, 0, sizeof(status_fields_));
}

bool Driver::requestStatusMessages()
{
  gateway_.queue(Message(LM_API_STATUS_VOLTOUT | device_number_));
  gateway_.queue(Message(LM_API_STATUS_VOLTBUS | device_number_));
  gateway_.queue(Message(LM_API_STATUS_CURRENT | device_number_));
  gateway_.queue(Message(LM_API_STATUS_TEMP | device_number_));
  gateway_.queue(Message(LM_API_STATUS_POS | device_number_));
  gateway_.queue(Message(LM_API_STATUS_SPD | device_number_));
  //gateway_.queue(Message(LM_API_STATUS_FAULT | device_number_));
  //gateway_.queue(Message(LM_API_STATUS_POWER | device_number_));
  //gateway_.queue(Message(LM_API_STATUS_CMODE | device_number_));
  //gateway_.queue(Message(LM_API_STATUS_VOUT | device_number_));
  return true;
}

float Driver::lastDutyCycle()
{
  StatusField* field = statusFieldForMessage(Message(LM_API_STATUS_VOLTOUT));
  return field->interpretFixed8x8() / 128.0;
}

float Driver::lastBusVoltage()
{
  StatusField* field = statusFieldForMessage(Message(LM_API_STATUS_VOLTBUS));
  return field->interpretFixed8x8();
}

float Driver::lastCurrent()
{
  StatusField* field = statusFieldForMessage(Message(LM_API_STATUS_CURRENT));
  return field->interpretFixed8x8();
}

Driver::StatusField* Driver::statusFieldForMessage(const Message& msg)
{
  // If it's not a STATUS message, there is no status field box to return.
  if (msg.getApi() & CAN_MSGID_API_M != CAN_API_MC_STATUS)
  {
    return NULL;
  }

  uint32_t status_field_index = (msg.getApi() & CAN_MSGID_API_ID_M) >> CAN_MSGID_API_S;
  return &status_fields_[status_field_index];
}

}
