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

#include "libreach/driver.hpp"

#include <iostream>
#include <ranges>
#include <stdexcept>
#include <utility>

namespace libreach
{

namespace
{

template <typename T>
auto inline convert_to_bytes(T value) -> std::vector<std::uint8_t>
{
  std::vector<std::uint8_t> bytes(sizeof(value));
  std::memcpy(bytes.data(), &value, sizeof(value));
  return bytes;
}

auto inline merge_bytes(const std::vector<std::uint8_t> & first, const std::vector<std::uint8_t> & second)
  -> std::vector<std::uint8_t>
{
  std::vector<std::uint8_t> bytes(first.size() + second.size());

  std::ranges::copy(first, std::back_inserter(bytes));
  std::ranges::copy(second, std::back_inserter(bytes));

  return bytes;
}

auto inline merge_bytes(const std::vector<std::vector<std::uint8_t>> & byte_vectors) -> std::vector<std::uint8_t>
{
  std::vector<std::uint8_t> bytes;

  for (const auto & byte_vector : byte_vectors) {
    std::ranges::copy(byte_vector, std::back_inserter(bytes));
  }

  return bytes;
}

}  // namespace

ReachDriver::ReachDriver(std::unique_ptr<protocol::Client> client, std::size_t q_size, std::size_t n_workers)
: client_(std::move(client)),
  packets_(boost::circular_buffer<Packet>(q_size))
{
  running_.store(true);

  packet_threads_.reserve(n_workers);
  for (std::size_t i = 0; i < n_workers; ++i) {
    packet_threads_.emplace_back([this] {
      while (running_.load()) {
        process_packet();
      }
    });
  }

  request_scheduler_thread_ = std::thread([this] {
    while (running_.load()) {
      process_requests();
    }
  });
}

ReachDriver::~ReachDriver()
{
  running_.store(false);

  packets_cv_.notify_all();
  for (auto & thread : packet_threads_) {
    if (thread.joinable()) {
      thread.join();
    }
  }

  request_cv_.notify_all();
  if (request_scheduler_thread_.joinable()) {
    request_scheduler_thread_.join();
  }
}

auto ReachDriver::set_mode(std::uint8_t device_id, Mode mode) const -> void
{
  send_packet(PacketId::MODE, device_id, {std::to_underlying(mode)});
}

auto ReachDriver::set_joint_velocity(std::uint8_t device_id, float velocity) const -> void
{
  send_packet(PacketId::VELOCITY, device_id, convert_to_bytes<float>(velocity));
}

auto ReachDriver::set_joint_position(std::uint8_t device_id, float position) const -> void
{
  send_packet(PacketId::POSITION, device_id, convert_to_bytes<float>(position));
}

auto ReachDriver::set_ee_pose(std::uint8_t device_id, float x, float y, float z, float rz, float ry, float rx) const
  -> void
{
  send_packet(
    PacketId::KM_END_POS,
    device_id,
    merge_bytes(
      {convert_to_bytes<float>(x),
       convert_to_bytes<float>(y),
       convert_to_bytes<float>(z),
       convert_to_bytes<float>(rz),
       convert_to_bytes<float>(ry),
       convert_to_bytes<float>(rx)}));
}

auto ReachDriver::set_ee_pose(std::uint8_t device_id, float x, float y, float z) const -> void
{
  send_packet(
    PacketId::KM_END_POS,
    device_id,
    merge_bytes(
      {convert_to_bytes<float>(x),
       convert_to_bytes<float>(y),
       convert_to_bytes<float>(z),
       convert_to_bytes<float>(0.0F),
       convert_to_bytes<float>(0.0F),
       convert_to_bytes<float>(0.0F)}));
}

auto ReachDriver::set_ee_velocity(std::uint8_t device_id, float vx, float vy, float vz, float vrz, float vry, float vrx)
  const -> void
{
  send_packet(
    PacketId::KM_END_VEL,
    device_id,
    merge_bytes(
      {convert_to_bytes<float>(vx),
       convert_to_bytes<float>(vy),
       convert_to_bytes<float>(vz),
       convert_to_bytes<float>(vrz),
       convert_to_bytes<float>(vry),
       convert_to_bytes<float>(vrx)}));
}

auto ReachDriver::set_ee_velocity(std::uint8_t device_id, float vx, float vy, float vz) const -> void
{
  send_packet(
    PacketId::KM_END_VEL,
    device_id,
    merge_bytes(
      {convert_to_bytes<float>(vx),
       convert_to_bytes<float>(vy),
       convert_to_bytes<float>(vz),
       convert_to_bytes<float>(0.0F),
       convert_to_bytes<float>(0.0F),
       convert_to_bytes<float>(0.0F)}));
}

auto ReachDriver::set_local_ee_velocity(
  std::uint8_t device_id,
  float vx,
  float vy,
  float vz,
  float vrz,
  float vry,
  float vrx) const -> void
{
  send_packet(
    PacketId::KM_END_VEL_LOCAL,
    device_id,
    merge_bytes(
      {convert_to_bytes<float>(vx),
       convert_to_bytes<float>(vy),
       convert_to_bytes<float>(vz),
       convert_to_bytes<float>(vrz),
       convert_to_bytes<float>(vry),
       convert_to_bytes<float>(vrx)}));
}

auto ReachDriver::set_relative_position(std::uint8_t device_id, float relative_position) const -> void
{
  send_packet(PacketId::INDEXED_RELATIVE_POSITION, device_id, convert_to_bytes<float>(relative_position));
}

auto ReachDriver::set_current(std::uint8_t device_id, float current) const -> void
{
  send_packet(PacketId::CURRENT, device_id, convert_to_bytes<float>(current));
}

auto ReachDriver::set_position_limits(std::uint8_t device_id, float min_position, float max_position) const -> void
{
  auto range = merge_bytes(convert_to_bytes<float>(min_position), convert_to_bytes<float>(max_position));
  send_packet(PacketId::POSITION_LIMITS, device_id, range);
}

auto ReachDriver::set_velocity_limits(std::uint8_t device_id, float min_velocity, float max_velocity) const -> void
{
  auto range = merge_bytes(convert_to_bytes<float>(min_velocity), convert_to_bytes<float>(max_velocity));
  send_packet(PacketId::VELOCITY_LIMITS, device_id, range);
}

auto ReachDriver::set_current_limits(std::uint8_t device_id, float min_current, float max_current) const -> void
{
  auto range = merge_bytes(convert_to_bytes<float>(min_current), convert_to_bytes<float>(max_current));
  send_packet(PacketId::CURRENT_LIMITS, device_id, range);
}

auto ReachDriver::request(PacketId packet_id, std::uint8_t device_id) -> std::future<Packet>
{
  std::promise<Packet> response;
  auto future = response.get_future();

  pending_requests_[packet_id].emplace_back(std::move(response));
  send_packet(PacketId::REQUEST, device_id, {std::to_underlying(packet_id)});

  return future;
}

auto ReachDriver::request_at_rate(PacketId packet_id, std::uint8_t device_id, std::chrono::milliseconds rate) const
  -> void
{
  {
    const std::lock_guard<std::mutex> lock(request_lock_);
    auto packet = Packet(PacketId::REQUEST, device_id, {std::to_underlying(packet_id)});
    requests_.emplace_back(packet, rate);
  }
  request_cv_.notify_all();
}

auto ReachDriver::request_at_rate(
  const std::vector<PacketId> & packet_ids,
  std::uint8_t device_id,
  std::chrono::milliseconds rate) const -> void
{
  if (packet_ids.empty()) {
    throw std::invalid_argument("Cannot request packets with an empty list of packet IDs.");
  }

  if (packet_ids.size() > 10) {
    throw std::invalid_argument("Cannot request more than 10 packets at a time.");
  }

  {
    const std::lock_guard<std::mutex> lock(request_lock_);

    std::vector<std::uint8_t> request_types(packet_ids.size());

    for (auto id : packet_ids) {
      request_types.push_back(std::to_underlying(id));
    }

    auto packet = Packet(PacketId::REQUEST, device_id, request_types);
    requests_.emplace_back(packet, rate);
  }
  request_cv_.notify_all();
}

auto ReachDriver::register_callback(PacketId packet_id, std::function<void(const Packet &)> && callback) -> void
{
  callbacks_[packet_id].emplace_back(std::move(callback));
}

auto ReachDriver::send_packet(const Packet & packet) const -> void  // NOLINT
{
  if (!client_->connected()) {
    throw std::runtime_error("Unable to send packet. Client is not connected.");
  }

  const std::lock_guard<std::mutex> lock(send_packet_lock_);
  client_->send_packet(packet);
}

auto ReachDriver::send_packet(PacketId packet_id, std::uint8_t device_id, const std::vector<std::uint8_t> & data) const
  -> void
{
  send_packet(Packet(packet_id, device_id, data));
}

auto ReachDriver::receive_packet(const Packet & packet) -> void
{
  {
    const std::lock_guard<std::mutex> lock(packets_lock_);
    packets_.push_back(packet);
  }
  packets_cv_.notify_one();
}

auto ReachDriver::receive_packets(const std::vector<Packet> & packets) -> void
{
  {
    const std::lock_guard<std::mutex> lock(packets_lock_);
    std::ranges::copy(packets, std::back_inserter(packets_));
  }
  packets_cv_.notify_all();
}

auto ReachDriver::process_packet() -> void
{
  std::unique_lock<std::mutex> lock(packets_lock_);
  packets_cv_.wait(lock, [this] { return !packets_.empty() || !running_.load(); });

  if (!running_.load()) {
    return;
  }

  const Packet packet = std::move(packets_.front());
  packets_.pop_front();

  lock.unlock();

  // Execute the callbacks for the packet
  auto it = callbacks_.find(packet.packet_id());
  if (it != callbacks_.end()) {
    for (const auto & callback : it->second) {
      callback(packet);
    }
  }

  // Resolve any pending requests
  if (pending_requests_.contains(packet.packet_id()) && !pending_requests_.at(packet.packet_id()).empty()) {
    pending_requests_[packet.packet_id()].front().set_value(packet);
    pending_requests_[packet.packet_id()].pop_front();
  }
}

auto ReachDriver::process_requests() -> void
{
  std::unique_lock<std::mutex> lock(request_lock_);
  request_cv_.wait(lock, [this] { return !requests_.empty() || !running_.load(); });

  if (!running_.load()) {
    return;
  }

  auto now = std::chrono::steady_clock::now();

  for (auto & request : requests_) {
    if (request.next_request <= now) {
      send_packet(request.packet);
      request.next_request = now + request.rate;
    }
  }

  // Wait for the earliest request to be ready
  request_cv_.wait_until(lock, std::ranges::min_element(requests_, [](const Request & a, const Request & b) {
                                 return a.next_request < b.next_request;
                               })->next_request);
}

}  // namespace libreach
