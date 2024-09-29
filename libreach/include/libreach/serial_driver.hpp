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

#include <chrono>
#include <cstddef>
#include <string>

#include "libreach/driver.hpp"
#include "libreach/serial_client.hpp"

namespace libreach
{

class SerialDriver : public ReachDriver
{
public:
  /// Create a new serial driver using:
  ///   - a serial port for communication (e.g., "/dev/ttyUSB0"),
  ///   - a queue size for storing incoming packets,
  ///   - a number of worker threads for processing incoming packets,
  ///   - a session timeout for heartbeat monitoring.
  explicit SerialDriver(
    const std::string & port,
    std::size_t q_size = 100,
    std::size_t n_workers = 1,
    std::chrono::seconds session_timeout = std::chrono::seconds(3));

  ~SerialDriver() = default;
};

}  // namespace libreach
