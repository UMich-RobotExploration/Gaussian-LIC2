#pragma once

#include <memory>
#include <string>
#include <vector>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace cocolic
{
  // Turns the raw Ouster packets of a bag (/ouster/lidar_packets, /ouster/imu_packets) into the point clouds and IMU
  // messages ouster-ros publishes for them, so such bags need no replay through the driver. Same as the driver with
  // timestamp_mode TIME_FROM_PTP_1588 (ptp_utc_tai_offset -37 s), point_cloud_frame os_lidar and point_type original,
  // organized and destaggered: the first return, x y z intensity t reflectivity ring ambient range.
  // IMU measurements from before the first lidar packet are dropped: when a recording starts, the sensor sends a
  // backlog of several seconds of them (the driver loses that burst in its queue, so replayed bags have none either).
  class OusterPacketDecoder
  {
  public:
    explicit OusterPacketDecoder(const std::string &metadata_json);  // the sensor metadata (/ouster/metadata)
    ~OusterPacketDecoder();

    // the scan this packet completes, nullptr while it is not complete
    sensor_msgs::msg::PointCloud2::SharedPtr AddLidarPacket(const std::vector<uint8_t> &buf);
    std::vector<sensor_msgs::msg::Imu> AddImuPacket(const std::vector<uint8_t> &buf);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
  };
} // namespace cocolic
