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

/// This example demonstrates the different approaches that can be taken to request data from a manipulator.
auto main() -> int
{
  const std::string serial_port = "/dev/ttyUSB0";
  libreach::SerialDriver driver(serial_port);

  driver.register_callback(libreach::PacketId::POSITION, [](const libreach::Packet & packet) {
    std::cout << "Received POSITION packet with value: " << libreach::deserialize<float>(packet) << "\n";
  });

  driver.register_callback(libreach::PacketId::VELOCITY, [](const libreach::Packet & packet) {
    std::cout << "Received VELOCITY packet with value: " << libreach::deserialize<float>(packet) << "\n";
  });

  driver.register_callback(libreach::PacketId::CURRENT, [](const libreach::Packet & packet) {
    std::cout << "Received CURRENT packet with value: " << libreach::deserialize<float>(packet) << "\n";
  });

  // The `request_at_rate` interface is designed to facilitate asynchronous periodic requests for data. This is
  // especially useful in scenarios where we want to monitor the state of the robot in real-time without needing to
  // manually request that data on our own.

  // Request a POSITION packet from joint A at a rate of 10 Hz (every 100 ms)
  driver.request_at_rate(libreach::PacketId::POSITION, std::to_underlying(libreach::Alpha5DeviceId::JOINT_A), 100ms);

  // Request VELOCITY and CURRENT packets from joint B at a rate of 5 Hz (every 200 ms)
  driver.request_at_rate(
    {libreach::PacketId::VELOCITY, libreach::PacketId::CURRENT},
    std::to_underlying(libreach::Alpha5DeviceId::JOINT_B),
    200ms);

  // The `request` interface is designed for one-time requests for data (e.g., for current configurations) and can be
  // used either synchronously or asynchronously.

  // Send a one-time request for the end effector pose
  std::future<libreach::Packet> f =
    driver.request(libreach::PacketId::KM_END_POS, std::to_underlying(libreach::Alpha5DeviceId::BASE));

  // Wait for the response - we could also wait for this asynchronously by spinning up a new thread
  const libreach::Packet packet = f.get();

  // Convert the packet data to a vector of floats
  const auto pose = libreach::deserialize<std::vector<float>>(packet);

  std::cout << "Received KM_END_POS packet. End effector position is: \n\r x: " << pose[0] << "\n\r y: " << pose[1]
            << "\n\r z: " << pose[2] << "\n";

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
