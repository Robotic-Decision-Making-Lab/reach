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
enum class PacketId : std::uint8_t
{
  REQUEST = 0x60,
  HEARTBEAT_SET = 0x91,
  HEARTBEAT_FREQUENCY = 0x92,
  MODE = 0x01,
  VELOCITY = 0X02,
  POSITION = 0x03,
  INDEXED_RELATIVE_POSITION = 0x0D,
  CURRENT = 0x05,
  KM_END_POS = 0xA1,
  KM_END_VEL = 0xA2,
  KM_END_VEL_LOCAL = 0xCB,
  KM_END_VEL_LOCAL_ROLL = 0xF4,
  ATI_FT_READING = 0xD8,
  POSITION_PRESET_GO = 0x55,
  POSITION_PRESET_CAPTURE = 0x56,
  POSITION_PRESET_SET_01 = 0x57,
  POSITION_PRESET_SET_02 = 0x58,
  POSITION_PRESET_SET_03 = 0x59,
  POSITION_PRESET_SET_04 = 0x5A,
  POSITION_PRESET_NAME_01 = 0x5B,
  POSITION_PRESET_NAME_02 = 0x5C,
  POSITION_PRESET_NAME_03 = 0x5D,
  POSITION_PRESET_NAME_04 = 0x5E,
  SAVE = 0x50,
  POSITION_LIMITS = 0x10,
  VELOCITY_LIMITS = 0x11,
  CURRENT_LIMITS = 0x12,
  KM_BOX_OBSTACLE_01 = 0xA5,
  KM_BOX_OBSTACLE_02 = 0xA6,
  KM_BOX_OBSTACLE_03 = 0xA7,
  KM_BOX_OBSTACLE_04 = 0xA8,
  KM_CYLINDER_OBSTACLE_01 = 0xAB,
  KM_CYLINDER_OBSTACLE_02 = 0xAC,
  KM_CYLINDER_OBSTACLE_03 = 0xAD,
  KM_CYLINDER_OBSTACLE_04 = 0xAE,
  VOLTAGE = 0x90,
  INTERNAL_HUMIDITY = 0x65,
  INTERNAL_TEMPERATURE = 0x66,
  INTERNAL_PRESSURE = 0x67,
  FACTORY_CLIMATE = 0x28,
  SOFTWARE_VERSION = 0x6C,
  HARDWARE_STATUS = 0x68,
};

}  // namespace libreach
