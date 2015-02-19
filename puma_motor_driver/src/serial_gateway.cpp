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

#include "puma_motor_driver/serial_gateway.h"
#include "ros/ros.h"

namespace puma_motor_driver
{

bool SerialGateway::connect()
{
  if (!serial_.isOpen())
  {
    try
    {
      serial_.open();
    }
    catch (std::exception& e)
    {
      ROS_ERROR("Exception thrown trying to open %s: %s", serial_.getPort().c_str(), e.what());
      return false;
    }
    return true;
  }
  return false;
}

bool SerialGateway::isConnected()
{
  return serial_.isOpen();
}

void SerialGateway::send(const Message& msg)
{
  ROS_DEBUG_NAMED("serial", "Sending ID 0x%08x, data (%d)", msg.id, msg.len);

  char unencoded_buffer[12];

  // CAN ID in little endian.
  unencoded_buffer[0] = msg.id;
  unencoded_buffer[1] = msg.id >> 8;
  unencoded_buffer[2] = msg.id >> 16;
  unencoded_buffer[3] = msg.id >> 24;

  // CAN data.
  memcpy(&unencoded_buffer[4], msg.data, msg.len);

  // Start of frame, packet length.
  uint8_t encoded_buffer[24];
  int encoded_buffer_index = 0;
  encoded_buffer[encoded_buffer_index++] = 0xff;
  encoded_buffer[encoded_buffer_index++] = 4 + msg.len;

  // Copy in ID and data, encoding as necessary.
  for (int unencoded_index = 0; unencoded_index < (4 + msg.len); unencoded_index++)
  {
    if (unencoded_buffer[unencoded_index] == '\xff')
    {
      encoded_buffer[encoded_buffer_index++] = '\xfe';
      encoded_buffer[encoded_buffer_index++] = '\xfe';
    }
    else if (unencoded_buffer[unencoded_index] == '\xfe')
    {
      encoded_buffer[encoded_buffer_index++] = '\xfe';
      encoded_buffer[encoded_buffer_index++] = '\xfd';
    }
    else
    {
      encoded_buffer[encoded_buffer_index++] = unencoded_buffer[unencoded_index];
    }
  }

  //for (int a = 0; a < encoded_buffer_index; a++)
  //  ROS_INFO("  0x%02x", (uint8_t)encoded_buffer[a]);

  try
  {
    size_t written = serial_.write(encoded_buffer, encoded_buffer_index);
    ROS_WARN_STREAM_COND(written < encoded_buffer_index, "Write to serial port timed out. Tried to write " <<
        encoded_buffer_index << "chars, instead wrote only " << written << ".");
  }
  catch (std::exception& e)
  {
    ROS_ERROR("Exception thrown trying to write to %s: %s", serial_.getPort().c_str(), e.what());
    serial_.close();
  }
}

bool SerialGateway::recv(Message* msg, uint32_t timeout_millis)
{

  return true;
}

}
