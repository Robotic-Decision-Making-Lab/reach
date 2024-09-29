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

#include "libreach/serial_client.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <iostream>
#include <ranges>
#include <stdexcept>

namespace libreach::protocol
{

SerialClient::SerialClient(
  const std::string & port,
  std::function<void(const std::vector<Packet> &)> && callback,
  std::chrono::seconds session_timeout,
  std::uint16_t max_bytes_to_read)
: Client(std::forward<std::function<void(const std::vector<Packet> &)>>(callback), session_timeout)
{
  if (port.empty()) {
    throw std::invalid_argument("Attempted to open file using an empty file path.");
  }

  handle_ = open(port.c_str(), O_RDWR | O_NOCTTY);

  if (handle_ < 0) {
    throw std::runtime_error(
      "Could not open specified serial device. Please verify that the device is not already being used and that you"
      " have configured read/write permissions.");
  }

  struct termios tty;

  if (tcgetattr(handle_, &tty) < 0) {
    throw std::runtime_error("Unable to get the current terminal configurations.");
  }

  // Set the baudrate to 115200
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  tty.c_cflag |= (CLOCAL | CREAD);         // Enable the receiver and set the mode to local mode
  tty.c_cflag &= ~CSIZE;                   // Mask the character size bits
  tty.c_cflag |= CS8;                      // Use 8 data bits per byte
  tty.c_cflag &= ~PARENB;                  // Disable parity
  tty.c_cflag &= ~CSTOPB;                  // Only one stop bit is used
  tty.c_cflag &= ~CRTSCTS;                 // Disable RTS/CTS hardware flow control
  tty.c_lflag &= ~ICANON;                  // Disable canonical input
  tty.c_lflag &= ~ECHO;                    // Disable echo
  tty.c_lflag &= ~ECHOE;                   // Disable erasure
  tty.c_lflag &= ~ECHONL;                  // Disable new-line echo
  tty.c_lflag &= ~ISIG;                    // Disable signals
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off IO control
  tty.c_iflag &=
    ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);  // Disable special handling of received bytes

  tty.c_oflag &= ~OPOST;  // Disable output post-processing
  tty.c_oflag &= ~ONLCR;  // Disable conversion of newline to carriage return

  tty.c_cc[VTIME] = 1;  // Set the timeout to 0.1s
  tty.c_cc[VMIN] = max_bytes_to_read;

  // Save the configurations
  if (tcsetattr(handle_, TCSANOW, &tty) != 0) {
    throw std::runtime_error("Unable to save the terminal configurations.");
  }

  start_polling_connection(max_bytes_to_read);
}

SerialClient::~SerialClient()
{
  shutdown_client();
  close(handle_);
}

auto SerialClient::write_to_connection(const std::vector<std::uint8_t> & data) const -> ssize_t
{
  return write(handle_, data.data(), data.size());
}

auto SerialClient::read_bytes(std::deque<std::uint8_t> & buffer, std::size_t n_bytes) const -> ssize_t
{
  std::vector<std::uint8_t> data(n_bytes);
  const ssize_t bytes_read = read(handle_, data.data(), n_bytes);
  std::ranges::copy(data | std::views::take(bytes_read), std::back_inserter(buffer));

  return bytes_read;
}

}  // namespace libreach::protocol
