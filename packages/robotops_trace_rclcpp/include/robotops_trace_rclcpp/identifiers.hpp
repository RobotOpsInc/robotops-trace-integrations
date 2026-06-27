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

#ifndef ROBOTOPS_TRACE_RCLCPP__IDENTIFIERS_HPP_
#define ROBOTOPS_TRACE_RCLCPP__IDENTIFIERS_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "rclcpp_action/types.hpp"
#include "rmw/types.h"

/// \file identifiers.hpp
/// \brief Canonical, cross-language string formatting for the identifiers used
/// as span correlation keys (the action goal UUID, the publisher GID).
///
/// The formatting here is the *contract*: every RobotOps integration that emits
/// these keys (rclcpp here, rclpy in ROB-423) MUST format them identically, or
/// the correlation agent (ROB-427) cannot join across the process boundary.

namespace robotops::trace::rclcpp
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

/// Format a 16-byte rclcpp_action goal UUID as the canonical RFC-4122
/// 8-4-4-4-12 lowercase hyphenated string (e.g.
/// "f47ac10b-58cc-4372-a567-0e02b2c3d479").
///
/// We deliberately do NOT use `rclcpp_action::to_string()` (32 un-hyphenated hex
/// chars): the hyphenated form is the language-neutral canonical UUID rendering
/// the Python integration and the agent join on. Both client and server observe
/// the same 16 bytes for a goal, so both emit the same string — that identity is
/// what makes the cross-process action hop deterministic.
inline std::string goal_id_to_string(const ::rclcpp_action::GoalUUID & uuid)
{
  const std::string h = detail::to_hex(uuid.data(), uuid.size());
  // 8-4-4-4-12
  return h.substr(0, 8) + "-" + h.substr(8, 4) + "-" + h.substr(12, 4) + "-" +
         h.substr(16, 4) + "-" + h.substr(20, 12);
}

/// Format an rmw publisher GID as lowercase hex (best-effort content key).
inline std::string gid_to_string(const rmw_gid_t & gid)
{
  return detail::to_hex(gid.data, RMW_GID_STORAGE_SIZE);
}

}  // namespace robotops::trace::rclcpp

#endif  // ROBOTOPS_TRACE_RCLCPP__IDENTIFIERS_HPP_
