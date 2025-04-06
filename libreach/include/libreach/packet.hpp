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

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

#include "libreach/packet_id.hpp"

namespace libreach
{

class Packet
{
public:
  /// Create a new packet given the packet ID, device ID, and (unencoded) data.
  Packet(PacketId packet_id, std::uint8_t device_id, std::vector<std::uint8_t> data);

  [[nodiscard]] auto packet_id() const -> PacketId;

  [[nodiscard]] auto device_id() const -> std::uint8_t;

  [[nodiscard]] auto data() const -> std::vector<std::uint8_t>;

  [[nodiscard]] auto data_size() const -> std::size_t;

  [[nodiscard]] auto pop_front(std::size_t bytes) -> std::vector<std::uint8_t>;

  [[nodiscard]] auto pop_back(std::size_t bytes) -> std::vector<std::uint8_t>;

private:
  PacketId packet_id_;
  std::uint8_t device_id_;
  std::vector<std::uint8_t> data_;
};

/// Deserialize a vector of bytes into a given type.
template <typename T>
[[nodiscard]] inline auto deserialize(std::vector<std::uint8_t> data) -> T
{
  if (data.size() != sizeof(T)) {
    throw std::invalid_argument("Cannot deserialize packet data into the requested type due to mismatched sizes.");
  }
  return *reinterpret_cast<const T *>(data.data());
}

template <>
[[nodiscard]] inline auto deserialize<std::vector<float>>(std::vector<std::uint8_t> data) -> std::vector<float>
{
  if (data.size() % sizeof(float) != 0) {
    throw std::invalid_argument("Cannot deserialize packet data into a vector of floats due to a mismatched size.");
  }

  std::vector<float> result;
  result.reserve(data.size() / sizeof(float));

  for (std::size_t i = 0; i < data.size(); i += sizeof(float)) {
    result.push_back(deserialize<float>({data.begin() + i, data.begin() + i + sizeof(float)}));
  }

  return result;
}

/// Deserialize a packet's data into a given type.
template <typename T>
[[nodiscard]] inline auto deserialize(const Packet & packet) -> T
{
  return deserialize<T>(packet.data());
}

namespace protocol
{
/// The delimiter used to separate packets in a byte stream.
const std::uint8_t PACKET_DELIMITER = 0x00;

/// Encode a packet into a byte stream.
auto encode_packet(const Packet & packet) -> std::vector<std::uint8_t>;

/// Decode a packet from a byte stream.
auto decode_packet(const std::vector<std::uint8_t> & data) -> Packet;

/// Decode multiple packets from a byte stream.
auto decode_packets(const std::vector<std::uint8_t> & data) -> std::vector<Packet>;

}  // namespace protocol

}  // namespace libreach
