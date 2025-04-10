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

#include <iostream>
#include <string>

#include "libreach/device_id.hpp"
#include "libreach/packet.hpp"
#include "libreach/packet_id.hpp"
#include "libreach/serial_driver.hpp"

using namespace std::chrono_literals;

// This example demonstrates the different approaches to sending packets to a robot using libreach.
// WARNING: This will set the joint position of joint A. Verify that the joint is clear of obstacles before running.
auto main() -> int
{
  const std::string serial_port = "/dev/ttyUSB0";

  // Create a new serial driver for the Alpha 5
  const libreach::SerialDriver driver(serial_port);

  // Set the desired position for joint E (rad)
  const auto joint_e = std::to_underlying(libreach::Alpha5DeviceId::JOINT_E);

  // 1. Helper functions have been provided to execute a variety of command commands such as setting the position
  const float desired_position = 0.53;
  driver.set_joint_position(joint_e, desired_position);

  // 2. If a helper function does not exist for a specific command, packets can be sent directly to the robot
  std::vector<std::uint8_t> bytes(sizeof(desired_position));
  std::memcpy(bytes.data(), &desired_position, sizeof(desired_position));

  driver.send_packet(libreach::PacketId::POSITION, joint_e, bytes);

  // 3. Packets can also be constructed and sent manually
  auto packet = libreach::Packet(libreach::PacketId::POSITION, joint_e, bytes);
  driver.send_packet(packet);

  // Let the driver run indefinitely
  while (true) {
    std::this_thread::sleep_for(1s);
  }

  return 0;
}
