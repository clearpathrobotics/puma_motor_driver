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

namespace puma_motor_driver
{

class Gateway;

class Driver
{
public:
  Driver(Gateway& gateway, uint8_t device_number, std::string device_name)
    : gateway_(gateway), device_number_(device_number), device_name_(device_name) {}

  void spinOnce();

  void voltageSet(float cmd);
  void speedSet(float cmd);
  void currentSet(float cmd);
  void positionSet(float cmd);

  /** Assignment operator, necessary on GCC 4.8 to copy instances
   *  into a vector. */
  Driver operator=(const Driver& rhs)
  {
    return Driver(gateway_, device_number_, device_name_);
  }

private:
  Gateway& gateway_;
  uint8_t device_number_;
  std::string device_name_;
};

}

#endif  // PUMA_MOTOR_DRIVER_DRIVER_H
