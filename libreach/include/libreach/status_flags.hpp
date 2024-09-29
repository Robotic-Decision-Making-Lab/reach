// Copyright (c) 2024 Evan Palmer
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), limited
// exclusively to use with products produced by Reach Robotics Pty Ltd, subject to
// the following conditions:
//
// The Software may only be used in conjunction with products manufactured or
// developed by Reach Robotics Pty Ltd.
//
// Redistributions or use of the Software in any other context, including but
// not limited to, integration, combination, or use with other products or
// software, are strictly prohibited without prior written authorization from Reach
// Robotics Pty Ltd.
//
// All copies of the Software, in whole or in part, must retain this notice and
// the above copyright notice.
//
// THIS SOFTWARE IS PROVIDED "AS IS," WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT. IN NO EVENT SHALL REACH ROBOTICS
// PTY LTD BE LIABLE FOR ANY CLAIM, DAMAGES, OR OTHER LIABILITY, WHETHER IN AN
// ACTION OF CONTRACT, TORT, OR OTHERWISE, ARISING FROM, OUT OF, OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#pragma once

#include <cstdint>

namespace libreach
{

/// Packet IDs for communication with Reach devices.
enum class ByteA : std::uint8_t
{
  FLASH_FAILED_READ = 0x80,
  HARDWARE_OVER_HUMIDITY = 0x40,
  HARDWARE_OVER_TEMPERATURE = 0x20,
  COMMS_SERIAL_ERROR = 0x10,
  COMMS_CRC_ERROR = 0x08,
  MOTOR_DRIVER_FAULT = 0x04,
  ENCODER_POSITION_ERROR = 0x02,
  ENCODER_NOT_DETECTED = 0x01,
};

enum class ByteB : std::uint8_t
{
  DEVICE_AXIS_CONFLICT = 0x80,
  MOTOR_NOT_CONNECTED = 0x40,
  MOTOR_OVER_CURRENT = 0x20,
  INNER_ENCODER_POSITION_ERROR = 0x10,
  DEVICE_ID_CONFLICT = 0x08,
  HARDWARE_OVER_PRESSURE = 0x04,
  MOTOR_DIRVER_OVER_CURRENT_AND_UNDER_VOLTAGE = 0x02,
  MOTOR_DRIVER_OVER_TEMPERATURE = 0x01,
};

enum class ByteC : std::uint8_t
{
  // 0x80 - unused
  // 0x40 - unused
  // 0x20 - unused
  JAW_ZERO_REQUIRED = 0x10,
  JOINT_SERVICE_DUE = 0x08,
  READ_PROTECTION_ENABLED = 0x04,
  ENCODER_FAULT = 0x02,
  // 0x01 - reserved
};

enum class ByteD : std::uint8_t
{
  ENCODER_POSITION_INVALID = 0x80,
  // 0x40 - unused
  // 0x20 - reserved
  LOW_SUPPLY_VOLTAGE = 0x10,
  INVALID_FIRMWARE = 0x08,
  // 0x04 - reserved
  CANBUS_ERROR = 0x02,
  POSITION_REPORT_NOT_RECEIVED = 0x01,
};

}  // namespace libreach
