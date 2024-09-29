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

#include <netinet/in.h>
#include <netinet/udp.h>
#include <sys/socket.h>

#include <cstdint>
#include <functional>
#include <string>

#include "libreach/client.hpp"

namespace libreach::protocol
{

class UdpClient : public Client
{
public:
  /// Create a new UDP client using an
  /// - IP address,
  /// - port,
  /// - packet callback,
  /// - session timeout,
  /// - and a maximum number of bytes to read on each poll.
  UdpClient(
    const std::string & addr,
    std::uint16_t port,
    std::function<void(const std::vector<Packet> &)> && callback,
    std::chrono::seconds session_timeout,
    std::uint16_t max_bytes_to_read = 64);

  ~UdpClient() override;

private:
  /// Read n bytes from the UDP socket into a buffer.
  auto read_bytes(std::deque<std::uint8_t> & buffer, std::size_t n_bytes) const -> ssize_t override;

  /// Write data to the UDP socket.
  auto write_to_connection(const std::vector<std::uint8_t> & data) const -> ssize_t override;

  int socket_;
};

}  // namespace libreach::protocol
