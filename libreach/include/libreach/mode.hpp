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

/// Operating modes for each device.
enum class Mode : std::uint8_t
{
  STANDBY = 0x00,
  DISABLE = 0x01,
  POSITION = 0x02,
  VELOCITY = 0x03,
  CURRENT = 0x04,
  INDEXED_RELATIVE_POSITION = 0x13,
  POSITION_PRESET = 0x14,
  ZERO_VELOCITY = 0x15,
  KINEMATICS_POSITION_BASE_FRAME = 0x17,
  KINEMATICS_VELOCITY_BASE_FRAME = 0x18,
  KINEMATICS_VELOCITY_END_EFFECTOR_FRAME = 0x1A,
  POSITION_VELOCITY = 0x1C,
  POSITION_HOLD = 0x1D,
  PASSIVE = 0x26,
};

}  // namespace libreach
