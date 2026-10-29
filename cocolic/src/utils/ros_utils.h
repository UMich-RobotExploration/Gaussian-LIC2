#pragma once

#include <cmath>
#include <cstdint>

#include <builtin_interfaces/msg/time.hpp>

// ROS 2 replacements for ros::Time's toSec() / toNSec() / fromSec() / fromNSec()

inline double stampToSec(const builtin_interfaces::msg::Time &stamp)
{
  return (double)stamp.sec + 1e-9 * (double)stamp.nanosec;
}

inline int64_t stampToNSec(const builtin_interfaces::msg::Time &stamp)
{
  return (int64_t)stamp.sec * 1000000000LL + (int64_t)stamp.nanosec;
}

inline builtin_interfaces::msg::Time stampFromNSec(int64_t t)
{
  int64_t sec = t / 1000000000LL;
  int64_t nsec = t % 1000000000LL;
  if (nsec < 0)
  {
    nsec += 1000000000LL;
    sec -= 1;
  }
  builtin_interfaces::msg::Time stamp;
  stamp.sec = (int32_t)sec;
  stamp.nanosec = (uint32_t)nsec;
  return stamp;
}

inline builtin_interfaces::msg::Time stampFromSec(double t)
{
  return stampFromNSec((int64_t)std::floor(t) * 1000000000LL +
                       (int64_t)std::round((t - std::floor(t)) * 1e9));
}
