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

#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>

#include "libreach/packet.hpp"

namespace libreach::protocol
{

class Client
{
public:
  /// Create a new client given a packet callback and a session timeout.
  Client(std::function<void(const std::vector<Packet> &)> && callback, std::chrono::seconds session_timeout);

  /// Destructor.
  virtual ~Client() = default;

  /// Check if the client is connected to the robot and running.
  [[nodiscard]] auto connected() const -> bool;

  /// Send a packet to the connected device.
  auto send_packet(const Packet & packet) const -> void;

protected:
  /// Start polling the connection; this should be called in the constructor of a derived class after connection.
  auto start_polling_connection(std::uint16_t max_bytes_to_read) -> void;

  /// Shutdown the client; this should be called in the destructor of a derived class before the connection is closed.
  auto shutdown_client() -> void;

private:
  enum class ConnectionState : std::uint8_t
  {
    CONNECTED,
    DISCONNECTED
  };

  /// Configure the heartbeat messages sent by the robot.
  auto enable_heartbeat(std::uint8_t frequency) const -> void;

  /// Disable the heartbeat message.
  auto disable_heartbeat() const -> void;

  /// Set the heartbeat frequency.
  auto set_heartbeat_rate(std::uint8_t frequency) const -> void;

  /// Set the timestamp that the last heartbeat was received.
  auto set_last_heartbeat(std::chrono::time_point<std::chrono::steady_clock> t) -> void;

  /// Check the heartbeat to ensure that the connection is still active.
  auto check_heartbeat(std::chrono::seconds timeout) -> void;

  /// Poll incoming data from the connection.
  auto poll_connection(std::uint16_t max_bytes_to_read) -> void;

  /// Read a specified number of bytes from the connection and push them onto the queue.
  virtual auto read_bytes(std::deque<std::uint8_t> & buffer, std::size_t n_bytes) const -> ssize_t = 0;

  /// Write data to a connection.
  virtual auto write_to_connection(const std::vector<std::uint8_t> & data) const -> ssize_t = 0;

  // Callback to execute when new packet(s) are received.
  std::function<void(const std::vector<Packet> &)> packet_callback_;

  std::atomic<bool> running_{false};

  // Monitor heartbeat messages from the robot to verify that the connection is still active.
  std::thread heartbeat_monitor_thread_;
  std::chrono::time_point<std::chrono::steady_clock> last_heartbeat_;
  mutable std::mutex last_heartbeat_lock_;

  mutable std::mutex connection_state_lock_;
  ConnectionState connection_state_{ConnectionState::DISCONNECTED};

  // Thread used to read incoming data.
  std::thread polling_thread_;
};

}  // namespace libreach::protocol
