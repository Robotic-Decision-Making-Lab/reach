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

#include <atomic>
#include <boost/circular_buffer.hpp>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>
#include <vector>

#include "libreach/client.hpp"
#include "libreach/mode.hpp"
#include "libreach/packet.hpp"
#include "libreach/packet_id.hpp"

namespace libreach
{

class ReachDriver
{
public:
  /// Create a new base driver using:
  ///   - a client (e.g., serial or TCP) for communication,
  ///   - a queue size for storing incoming packets,
  ///   - a number of worker threads for processing incoming packets.
  ReachDriver(std::unique_ptr<protocol::Client> client, std::size_t q_size, std::size_t n_workers);

  /// Set the operating mode of a device.
  auto set_mode(std::uint8_t device_id, Mode mode) const -> void;

  /// Set the desired velocity of a device (rad/s for rotational joints, mm/s for linear).
  auto set_joint_velocity(std::uint8_t device_id, float velocity) const -> void;

  /// Set the desired position of a device ([0, 2π] rad for rotational joints, mm for linear).
  auto set_joint_position(std::uint8_t device_id, float position) const -> void;

  /// Set the end-effector pose with respect to the base frame (mm for position, rad for rotation).
  ///
  /// The device ID should be set to the manipulator base.
  ///
  /// INFO: Inverse kinematics is not available for manipulators with less than 5 functions. Orbit controls (YPR) are
  /// not available for manipulators with less than 7 functions (values will be ignored). Additionally, inverse
  /// kinematics is disabled for standard manipulator packages.
  auto set_ee_pose(std::uint8_t device_id, float x, float y, float z, float rz, float ry, float rx) const -> void;

  /// Set the end-effector position with respect to the base frame (mm).
  ///
  /// The device ID should be set to the manipulator base.
  ///
  /// INFO: Inverse kinematics is not available for manipulators with less than 5 functions. Orbit controls (YPR) are
  /// not available for manipulators with less than 7 functions (values will be ignored). Additionally, inverse
  /// kinematics is disabled for standard manipulator packages.
  auto set_ee_pose(std::uint8_t device_id, float x, float y, float z) const -> void;

  /// Set the global end-effector velocity (mm/s for linear velocity, rad/s for angular velocity).
  ///
  /// The device ID should be set to the manipulator base.
  ///
  /// INFO: Inverse kinematics is not available for manipulators with less than 5 functions. Orbit controls (YPR) are
  /// not available for manipulators with less than 7 functions (values will be ignored). Additionally, inverse
  /// kinematics is disabled for standard manipulator packages.
  auto set_ee_velocity(std::uint8_t device_id, float vx, float vy, float vz, float vrz, float vry, float vrx) const
    -> void;

  /// Set the global linear velocity of the end-effector (mm/s).
  ///
  /// The device ID should be set to the manipulator base.
  ///
  /// INFO: Inverse kinematics is not available for manipulators with less than 5 functions. Orbit controls (YPR) are
  /// not available for manipulators with less than 7 functions (values will be ignored). Additionally, inverse
  /// kinematics is disabled for standard manipulator packages.
  auto set_ee_velocity(std::uint8_t device_id, float vx, float vy, float vz) const -> void;

  /// Set the local end-effector velocity (mm/s for linear velocity, rad/s for angular velocity).
  ///
  /// The device ID should be set to the manipulator base.
  ///
  /// INFO: Inverse kinematics is not available for manipulators with less than 5 functions. Orbit controls (YPR) are
  /// not available for manipulators with less than 7 functions (values will be ignored). Additionally, inverse
  /// kinematics is disabled for standard manipulator packages.
  auto set_local_ee_velocity(std::uint8_t device_id, float vx, float vy, float vz, float vrz, float vry, float vrx)
    const -> void;

  /// Set the desired relative position of a device ([0, 2π] rad for rotational joints, mm for linear).
  auto set_relative_position(std::uint8_t device_id, float relative_position) const -> void;

  /// Set the desired current of a device (mAh).
  auto set_current(std::uint8_t device_id, float current) const -> void;

  /// Set the minimum and maximum position of a joint ([0, 2π] rad for rotational joints, mm for linear joints).
  auto set_position_limits(std::uint8_t device_id, float min_position, float max_position) const -> void;

  /// Set the minimum and maximum velocity of a joint (rad/s for rotational joints, mm/s for linear joints).
  auto set_velocity_limits(std::uint8_t device_id, float min_velocity, float max_velocity) const -> void;

  /// Set the minimum and maximum current of a joint (mAh).
  auto set_current_limits(std::uint8_t device_id, float min_current, float max_current) const -> void;

  /// Move to a preset position.
  ///
  /// The device ID should be set to the manipulator base.
  ///
  /// INFO: This needs to be called at a rate of >10 Hz to continue driving to the preset position.
  auto go_to_preset_position(std::uint8_t device_id, std::uint8_t preset_id) const -> void;

  /// Save the current position as a position preset for the provided preset index.
  ///
  /// INFO: For manipulators with 5+ functions, the positions preset capture packet be sent to the Base device. For
  /// manipulators with 4 functions or fewer, the position preset capture packet must be sent to the individual axes
  /// one-by-one (the 0xFF device ID is not valid).
  auto capture_preset_position(std::uint8_t device_id, std::uint8_t preset_id) const -> void;

  /// Set the specified preset position angles given the preset id [0:3] (rad for rotational joints, mm for linear
  /// joints). Each position corresponds to one axis of the target device, starting from device ID 0x01.
  ///
  /// Unused positions should be set to 0.
  ///
  /// The device ID should be set to the manipulator base.
  auto set_preset_position(
    std::uint8_t device_id,
    std::uint8_t preset_id,
    float A,
    float B,
    float C,
    float D,
    float E,
    float F,
    float G,
    float H) const -> void;

  /// Set the name of the specified preset position given the preset ID [0:3].
  ///
  /// The name should be no larger than 8 bytes - anything larger will be truncated.
  ///
  /// The device ID should be set to the manipulator base.
  auto name_position_preset(std::uint8_t device_id, std::uint8_t preset_id, const std::string & name) const -> void;

  /// Save the current configuration to the device.
  ///
  /// This should be sent individually to each joint.
  auto save(std::uint8_t device_id) const -> void;

  /// Request a packet from the specified device.
  ///
  /// This should be used for one-time, low-frequency requests
  auto request(PacketId packet_id, std::uint8_t device_id) -> std::future<Packet>;

  /// Request a packet from the specified device at some rate.
  ///
  /// This can be used to request state information from the device at a fixed rate. When using this function,
  /// a callback should be added to process the incoming responses.
  ///
  /// WARNING: Exercise caution when using this function in conjunction with the `request` function. If the same packets
  /// are requested, the driver will send the packet to both the callback and as a future response.
  auto request_at_rate(PacketId packet_id, std::uint8_t device_id, std::chrono::milliseconds rate) const -> void;

  /// Request up to 10 packets from the specified device at some rate.
  ///
  /// This can be used to request state information from the device at a fixed rate. When using this function,
  /// a callback should be added to process the incoming responses.
  ///
  /// WARNING: Exercise caution when using this function in conjunction with the `request` function. If the same packets
  /// are requested, the driver will send the packet to both the callback and as a future response.
  auto request_at_rate(const std::vector<PacketId> & packet_ids, std::uint8_t device_id, std::chrono::milliseconds rate)
    const -> void;

  /// Register a callback for a specific packet ID.
  auto register_callback(PacketId packet_id, std::function<void(const Packet &)> && callback) -> void;

  /// Send a packet using the configured client.
  auto send_packet(const Packet & packet) const -> void;

  /// Send a packet using the configured client.
  auto send_packet(PacketId packet_id, std::uint8_t device_id, const std::vector<std::uint8_t> & data) const -> void;

  /// Get the Reach Serial Protocol version.
  static auto api_version() -> std::string;

protected:
  ~ReachDriver();

  /// Callback executed when a packet is received by a client.
  auto receive_packet(const Packet & packet) -> void;

  /// Callback executed when multiple packets have been received by a client.
  auto receive_packets(const std::vector<Packet> & packets) -> void;

  std::unique_ptr<protocol::Client> client_;

private:
  struct Request
  {
    Request(Packet packet, std::chrono::milliseconds rate)
    : packet(std::move(packet)),
      rate(rate),
      next_request(std::chrono::steady_clock::now())
    {
    }

    Packet packet;
    std::chrono::milliseconds rate;
    std::chrono::time_point<std::chrono::steady_clock> next_request;
  };

  /// Process the first packet in the packet queue.
  auto process_packet() -> void;

  /// Process the requests.
  auto process_requests() -> void;

  std::atomic<bool> running_{false};

  // Packets are stored to a circular buffer to avoid limit the amount of old data stored.
  boost::circular_buffer<Packet> packets_;
  std::mutex packets_lock_;
  std::condition_variable packets_cv_;
  std::vector<std::thread> packet_threads_;

  // Requests sent using the promise-future pattern.
  std::unordered_map<PacketId, std::deque<std::promise<Packet>>> pending_requests_;

  // request_at_rate is managed by a scheduler to ensure that requests are sent at the correct rate.
  mutable std::vector<Request> requests_;
  mutable std::mutex request_lock_;
  mutable std::condition_variable request_cv_;
  std::thread request_scheduler_thread_;

  // We need to manage access to the client to account for the request scheduler, which runs in its own thread.
  mutable std::mutex send_packet_lock_;

  std::unordered_map<PacketId, std::vector<std::function<void(Packet)>>> callbacks_;
};

}  // namespace libreach
