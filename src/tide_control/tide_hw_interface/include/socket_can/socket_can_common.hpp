// Copyright 2021 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef ROS2_SOCKETCAN__SOCKET_CAN_COMMON_HPP_
#define ROS2_SOCKETCAN__SOCKET_CAN_COMMON_HPP_

#include <sys/select.h>
#include <sys/time.h>
#include <linux/can.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace tide_hw_interface
{
/// Bind a non-blocking CAN_RAW socket to the given interface
/// \param[in] interface The name of the interface to bind, must be smaller than IFNAMSIZ
/// \param[in] enable_fd Whether this socket uses CAN FD or not
/// \return The file descriptor bound to the given interface
/// \throw std::runtime_error If one of socket(), fnctl(), ioctl(), bind() failed
/// \throw std::domain_error If the provided interface name is too long
int32_t bind_can_socket(const std::string& interface, bool enable_fd);

/// Set SocketCAN filters
/// \param[in] fd File descriptor of the socket
/// \param[in] f_list List of filters to be applied.
/// \throw std::runtime_error If filters couldn't be applied
void set_can_filter(int32_t fd, const std::vector<struct can_filter>& f_list);

/// Set SocketCAN error filter
/// \param[in] fd File descriptor of the socket
/// \param[in] err_mask Error mask to be applied as a filter
void set_can_err_filter(int32_t fd, can_err_mask_t err_mask);

/// Set filters joining option for SocketCAN. If set, all filters
/// must match for the frame to be passed.
/// \param[in] fd File descriptor of the socket
/// \param[in] join_filters Should the filters be joined?
void set_can_filter_join(int32_t fd, bool join_filters);

/// Convert std::chrono duration to timeval (with microsecond resolution)
struct timeval to_timeval(const std::chrono::nanoseconds timeout) noexcept;
/// Convert timeval to time in microseconds
uint64_t from_timeval(const struct timeval tv) noexcept;
/// Create a fd_set for use with select() that only contains the specified file descriptor
fd_set single_set(int32_t file_descriptor) noexcept;

}  // namespace tide_hw_interface

#endif  // ROS2_SOCKETCAN__SOCKET_CAN_COMMON_HPP_
