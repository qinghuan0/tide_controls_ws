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

#include "socket_can/socket_can_common.hpp"

#include <fcntl.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can/raw.h>

#include <unistd.h>
#include <linux/can.h>

#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace tide_hw_interface
{
////////////////////////////////////////////////////////////////////////////////
int32_t bind_can_socket(const std::string& interface, bool enable_fd)
{
  if (interface.length() >= static_cast<std::string::size_type>(IFNAMSIZ))
  {
    throw std::domain_error{ "CAN interface name too long" };
  }

  // Create file descriptor
  const auto file_descriptor = socket(PF_CAN, static_cast<int32_t>(SOCK_RAW), CAN_RAW);
  if (0 > file_descriptor)
  {
    throw std::runtime_error{ "Failed to open CAN socket" };
  }
  // Make it non-blocking so we can use timeouts
  // lint -e{9001} NOLINT I can't do anything about using this third party octal constant...
  if (0 != fcntl(file_descriptor, F_SETFL, O_NONBLOCK))
  {
    throw std::runtime_error{ "Failed to set CAN socket to nonblocking" };
  }

  // Set up address/interface name
  struct ifreq ifr;
  // The destination struct is local; don't need address
  (void)strncpy(&ifr.ifr_name[0U], interface.c_str(), interface.length() + 1U);
  if (0 != ioctl(file_descriptor, static_cast<uint32_t>(SIOCGIFINDEX), &ifr))
  {
    throw std::runtime_error{ "Failed to set CAN socket name via ioctl()" };
  }

  struct sockaddr_can addr;
  addr.can_family = static_cast<decltype(addr.can_family)>(AF_CAN);
  addr.can_ifindex = ifr.ifr_ifindex;

  // Bind address
  // lint -save -e586 NOLINT This (c-style casts actually) is the idiomatic way to use sockaddr
  if (0 > bind(file_descriptor, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)))
  {
    throw std::runtime_error{ "Failed to bind CAN socket" };
  }
  // lint -restore NOLINT

  // Enable CAN FD support
  const int32_t enable_canfd = enable_fd ? 1 : 0;
  if (0 != setsockopt(file_descriptor, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd,
                      sizeof(enable_canfd)))
  {
    throw std::runtime_error{ "Failed to enable CAN FD support" };
  }

  return file_descriptor;
}

////////////////////////////////////////////////////////////////////////////////
void set_can_filter(int32_t fd, const std::vector<struct can_filter>& f_list)
{
  if (0 != setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, f_list.empty() ? NULL : f_list.data(),
                      sizeof(can_filter) * f_list.size()))
  {
    throw std::runtime_error{ "Failed to set up CAN filters: " + std::string{ strerror(errno) } };
  }
}

////////////////////////////////////////////////////////////////////////////////
void set_can_err_filter(int32_t fd, can_err_mask_t err_mask)
{
  if (0 != setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)))
  {
    throw std::runtime_error{ "Failed to set up CAN error filters: " +
                              std::string{ strerror(errno) } };
  }
}

////////////////////////////////////////////////////////////////////////////////
void set_can_filter_join(int32_t fd, bool join_filters)
{
  auto join = static_cast<int>(join_filters);
  if (0 != setsockopt(fd, SOL_CAN_RAW, CAN_RAW_JOIN_FILTERS, &join, sizeof(join)))
  {
    throw std::runtime_error{ "Failed to set up joined CAN filters: " +
                              std::string{ strerror(errno) } };
  }
}

////////////////////////////////////////////////////////////////////////////////
struct timeval to_timeval(const std::chrono::nanoseconds timeout) noexcept
{
  const auto count = timeout.count();
  constexpr auto BILLION = 1'000'000'000LL;
  struct timeval c_timeout;
  c_timeout.tv_sec = static_cast<decltype(c_timeout.tv_sec)>(count / BILLION);
  c_timeout.tv_usec = static_cast<decltype(c_timeout.tv_usec)>((count % BILLION) / 1000LL);

  return c_timeout;
}

////////////////////////////////////////////////////////////////////////////////
uint64_t from_timeval(const struct timeval tv) noexcept
{
  return static_cast<uint64_t>(tv.tv_sec) * 1e6 + tv.tv_usec;
}

////////////////////////////////////////////////////////////////////////////////
fd_set single_set(int32_t file_descriptor) noexcept
{
  fd_set descriptor_set;
  // TODO(c.ho) sort through all these MISRA errors...
  // lint -save -e9146 NOLINT
  // lint --e{9063, 9036, 9084, 9027, 9033, 550, 717, 9001, 9093, 953} NOLINT
  FD_ZERO(&descriptor_set);
  // lint --e{9063, 9036, 9084, 9027, 9033, 550, 9123, 9125, 9126, 1924, 9130} NOLINT
  FD_SET(file_descriptor, &descriptor_set);
  // lint -restore NOLINT

  return descriptor_set;
}
}  // namespace tide_hw_interface
