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

#include "libreach/udp_client.hpp"

#include <arpa/inet.h>
#include <unistd.h>

#include <iostream>
#include <ranges>
#include <stdexcept>

namespace libreach::protocol
{

UdpClient::UdpClient(
  const std::string & addr,
  std::uint16_t port,
  std::function<void(const std::vector<Packet> &)> && callback,
  std::chrono::seconds session_timeout,
  std::uint16_t max_bytes_to_read)
: Client(std::forward<std::function<void(const std::vector<Packet> &)>>(callback), session_timeout)
{
  socket_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_ < 0) {
    throw std::runtime_error("Failed to open UDP socket");
  }

  struct sockaddr_in sockaddr;

  sockaddr.sin_family = AF_INET;
  sockaddr.sin_port = htons(port);
  sockaddr.sin_addr.s_addr = inet_addr(addr.c_str());

  if (sockaddr.sin_addr.s_addr == INADDR_NONE) {
    throw std::runtime_error("Invalid socket address " + addr);
  }

  if (connect(socket_, reinterpret_cast<struct sockaddr *>(&sockaddr), sizeof(sockaddr)) < 0) {
    throw std::runtime_error("Failed to connect to UDP socket");
  }

  start_polling_connection(max_bytes_to_read);
}

UdpClient::~UdpClient()
{
  shutdown_client();
  close(socket_);
}

auto UdpClient::write_to_connection(const std::vector<std::uint8_t> & data) const -> ssize_t
{
  return send(socket_, data.data(), data.size(), 0);
}

auto UdpClient::read_bytes(std::deque<std::uint8_t> & buffer, std::size_t n_bytes) const -> ssize_t
{
  std::vector<std::uint8_t> data(n_bytes);
  const ssize_t n_read = recv(socket_, data.data(), data.size(), 0);
  std::ranges::copy(data | std::views::take(n_read), std::back_inserter(buffer));

  return n_read;
}

}  // namespace libreach::protocol
