// Copyright (c) 2025 Evan Palmer
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

/// Reach Alpha 5 device IDs.
enum class Alpha5DeviceId : std::uint8_t
{
  JOINT_A = 0x01,
  JOINT_B = 0x02,
  JOINT_C = 0x03,
  JOINT_D = 0x04,
  JOINT_E = 0x05,
  BASE = 0x05,
  ALL_JOINTS = 0xFF,
};

/// Reach Bravo 7 device IDs.
enum class Bravo7DeviceId : std::uint8_t
{
  JOINT_A = 0x01,
  JOINT_B = 0x02,
  JOINT_C = 0x03,
  JOINT_D = 0x04,
  JOINT_E = 0x05,
  JOINT_F = 0x06,
  JOINT_G = 0x07,
  BASE = 0xD,
  COMPUTE = 0x0E,
  ALL_JOINTS = 0xFF,
  FORCE_TORQUE_SENSOR = 0x0D,
};

}  // namespace libreach
