// Copyright 2026 Robot Ops Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROBOTOPS_TRACE_ROS2_CONTROL__IDENTIFIERS_HPP_
#define ROBOTOPS_TRACE_ROS2_CONTROL__IDENTIFIERS_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include "rclcpp_action/types.hpp"

/// \file identifiers.hpp
/// \brief Canonical, cross-language string formatting for the action goal UUID.
///
/// This formatting is the *contract* that makes the cross-process action hop
/// deterministic: the rclcpp/rclpy action CLIENT and this ros2_control SERVER
/// both observe the same 16 goal-UUID bytes and MUST render them to the SAME
/// string, or the correlation agent (ROB-427) cannot stitch the controller hop
/// under the action client. It is therefore byte-for-byte identical to
/// `robotops_trace_rclcpp`'s `goal_id_to_string` (RFC-4122 8-4-4-4-12 lowercase).
/// We copy the ~10 lines rather than depend on the rclcpp integration package so
/// that this package's only ROS deps are the ros2_control framework + control
/// messages.

namespace robotops::trace::ros2_control
{

namespace detail
{

/// Lowercase-hex a byte range with no separators.
inline std::string to_hex(const std::uint8_t * bytes, std::size_t count)
{
  static const char digits[] = "0123456789abcdef";
  std::string out;
  out.reserve(count * 2);
  for (std::size_t i = 0; i < count; ++i) {
    out.push_back(digits[(bytes[i] >> 4) & 0x0F]);
    out.push_back(digits[bytes[i] & 0x0F]);
  }
  return out;
}

}  // namespace detail

/// Format a 16-byte action goal UUID as the canonical RFC-4122 8-4-4-4-12
/// lowercase hyphenated string (e.g. "f47ac10b-58cc-4372-a567-0e02b2c3d479").
///
/// Deliberately NOT `rclcpp_action::to_string()` (32 un-hyphenated hex chars):
/// the hyphenated form is the language-neutral canonical UUID rendering that the
/// Python integration and the agent join on. Identical bytes -> identical string
/// on both client and server == a deterministic cross-process action hop.
inline std::string goal_id_to_string(const ::rclcpp_action::GoalUUID & uuid)
{
  const std::string h = detail::to_hex(uuid.data(), uuid.size());
  // 8-4-4-4-12
  return h.substr(0, 8) + "-" + h.substr(8, 4) + "-" + h.substr(12, 4) + "-" +
         h.substr(16, 4) + "-" + h.substr(20, 12);
}

}  // namespace robotops::trace::ros2_control

#endif  // ROBOTOPS_TRACE_ROS2_CONTROL__IDENTIFIERS_HPP_
