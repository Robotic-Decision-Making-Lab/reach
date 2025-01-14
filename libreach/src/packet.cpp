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

#include "libreach/packet.hpp"

#include <algorithm>
#include <iostream>
#include <ranges>
#include <stdexcept>
#include <utility>

#include "cobs.hpp"
#include "crc.hpp"

namespace libreach
{

Packet::Packet(PacketId packet_id, std::uint8_t device_id, std::vector<std::uint8_t> data)
: packet_id_(packet_id),
  device_id_(device_id),
  data_(std::move(data))
{
  if (data_.empty()) {
    throw std::invalid_argument("Cannot create a packet with empty data.");
  }
}

auto Packet::packet_id() const -> PacketId { return packet_id_; }

auto Packet::device_id() const -> std::uint8_t { return device_id_; }

auto Packet::data() const -> std::vector<std::uint8_t> { return data_; }

auto Packet::data_size() const -> std::size_t { return data_.size(); }

auto Packet::pop_front(std::size_t bytes) -> std::vector<std::uint8_t>
{
  if (bytes > data_.size()) {
    throw std::invalid_argument("Cannot pop more bytes than are available in the packet.");
  }

  auto popped = data_ | std::views::take(bytes);
  data_.erase(data_.begin(), data_.begin() + bytes);

  return {popped.begin(), popped.end()};
}

auto Packet::pop_back(std::size_t bytes) -> std::vector<std::uint8_t>
{
  if (bytes > data_.size()) {
    throw std::invalid_argument("Cannot pop more bytes than are available in the packet.");
  }

  auto popped = data_ | std::views::drop(data_.size() - bytes);
  data_.erase(data_.end() - bytes, data_.end());

  return {popped.begin(), popped.end()};
}

namespace protocol
{

auto encode_packet(const Packet & packet) -> std::vector<std::uint8_t>
{
  std::vector<std::uint8_t> data(packet.data());

  data.push_back(std::to_underlying(packet.packet_id()));
  data.push_back(packet.device_id());

  // Length is the current buffer size plus two (length and CRC)
  data.push_back(data.size() + 2);

  data.push_back(calculate_crc(data));

  return encode_cobs(data);
}

auto decode_packet(const std::vector<std::uint8_t> & data) -> Packet
{
  if (data.empty()) {
    throw std::invalid_argument("Cannot decode an empty byte stream.");
  }

  std::vector<std::uint8_t> decoded_data = decode_cobs(data);

  if (decoded_data.empty()) {
    throw std::runtime_error("Decoded data is empty");
  }

  const std::uint8_t actual_crc = decoded_data.back();
  decoded_data.pop_back();

  const std::uint8_t expected_crc = calculate_crc(decoded_data);

  if (actual_crc != expected_crc) {
    throw std::runtime_error("The expected and actual CRC values do not match.");
  }

  const auto length = static_cast<std::vector<std::uint8_t>::size_type>(decoded_data.back());
  decoded_data.pop_back();

  if ((decoded_data.size() + 2) != length) {
    throw std::runtime_error("The specified payload size is not equal to the actual payload size.");
  }

  const std::uint8_t device_id = decoded_data.back();
  decoded_data.pop_back();

  const std::uint8_t packet_id = decoded_data.back();
  decoded_data.pop_back();

  return {static_cast<PacketId>(packet_id), device_id, decoded_data};
}

auto decode_packets(const std::vector<std::uint8_t> & data) -> std::vector<Packet>
{
  if (data.empty()) {
    throw std::invalid_argument("Cannot decode an empty buffer.");
  }

  std::vector<Packet> packets;

  auto start = data.begin();
  auto iter = std::find_if(start, data.end(), [](const std::uint8_t & b) { return b == PACKET_DELIMITER; });

  while (iter != data.end()) {
    const std::vector<std::uint8_t> packet_data(start, iter);

    start = iter + 1;
    iter = std::find_if(start, data.end(), [](const std::uint8_t & b) { return b == PACKET_DELIMITER; });

    if (packet_data.empty()) {
      continue;
    }

    try {
      const Packet packet = decode_packet(packet_data);
      packets.push_back(packet);
    }
    catch (const std::exception & e) {
      // TODO(evan-palmer): Right now the logging spams the console with errors. This should be refactored when
      // std::expected is integrated.
    }
  }

  return packets;
}

}  // namespace protocol

}  // namespace libreach
