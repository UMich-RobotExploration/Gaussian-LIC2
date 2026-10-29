#include "lidar/ouster_packet_decoder.h"

#ifdef COCOLIC_WITH_OUSTER

#include <algorithm>
#include <cstring>
#include <limits>

#include <ouster/lidar_scan.h>
#include <ouster/packet.h>
#include <ouster/types.h>
#include <ouster/xyzlut.h>
#include <ouster_ros/os_point.h>
#include <ouster_ros/os_ros.h>
#include <pcl_conversions/pcl_conversions.h>

namespace cocolic
{
  namespace sdk = ouster::sdk::core;

  namespace
  {
    constexpr int64_t kPtpUtcTaiOffsetNs = -37000000000LL;  // the driver's default ptp_utc_tai_offset
    constexpr uint32_t kMaxRangeMm = 100000000;              // the default max_range of its replay.launch.xml

    // a channel of the scan as uint32, whatever its width in the packet profile; zeros if the profile has none
    sdk::img_t<uint32_t> Channel(const sdk::LidarScan &ls, const std::string &name)
    {
      if (!ls.has_field(name))
        return sdk::img_t<uint32_t>::Zero(ls.h, ls.w);
      switch (ls.field_type(name).element_type)
      {
      case sdk::ChanFieldType::UINT8:
        return ls.field<uint8_t>(name).cast<uint32_t>();
      case sdk::ChanFieldType::UINT16:
        return ls.field<uint16_t>(name).cast<uint32_t>();
      case sdk::ChanFieldType::UINT32:
        return ls.field<uint32_t>(name);
      default:
        return ls.field<uint64_t>(name).cast<uint32_t>();
      }
    }

    // as in ouster-ros lidar_packet_handler.h, rounding included
    uint64_t LinearInterpolate(int x0, uint64_t y0, int x1, uint64_t y1, int x)
    {
      uint64_t min_v = std::min(y0, y1), max_v = std::max(y0, y1);
      double sign = y1 > y0 ? +1 : -1;
      return y0 + (x - x0) * sign * (max_v - min_v) / (x1 - x0);
    }
  } // namespace

  struct OusterPacketDecoder::Impl
  {
    sdk::SensorInfo info;
    std::shared_ptr<sdk::PacketFormat> format;
    sdk::ScanBatcher batcher;
    sdk::LidarScan scan;
    sdk::ArrayX3R<float> direction, offset;  // xyz = range * direction + offset, in os_lidar
    double col_spacing_ns;

    uint64_t first_lidar_ts = 0;  // of the first lidar packet, 0 until there is one

    bool first_scan = true;
    int last_nonzero_idx = 0;  // the last column of the previous scan with a timestamp
    uint64_t last_nonzero_ts = 0;

    explicit Impl(const std::string &metadata_json)
        : info(metadata_json), format(std::make_shared<sdk::PacketFormat>(sdk::get_format(info))),
          batcher(info), scan(info)
    {
      auto lut = sdk::make_xyz_lut(info.format.columns_per_frame, info.format.pixels_per_column, sdk::RANGE_UNIT,
                                   info.beam_to_lidar_transform, sdk::mat4d::Identity(),
                                   info.beam_azimuth_angles, info.beam_altitude_angles);
      direction = lut.direction.cast<float>();
      offset = lut.offset.cast<float>();
      auto mode = info.config.lidar_mode.value();
      col_spacing_ns = 1e9 / (sdk::n_cols_of_lidar_mode(mode) * sdk::frequency_of_lidar_mode(mode));
    }

    // the timestamp of the first column, filled in when it was not received (compute_scan_ts_0 / _n of ouster-ros);
    // false for a scan without any timestamp
    bool ScanTimestamp(const Eigen::Ref<sdk::LidarScan::Header<uint64_t>> &ts, uint64_t &scan_ts)
    {
      const uint64_t *begin = ts.data(), *end = ts.data() + ts.size();
      const uint64_t *first = std::find_if(begin, end, [](uint64_t t) { return t != 0; });
      if (first == end)
        return false;
      const int first_idx = first - begin;
      if (first_idx == 0)
        scan_ts = *first;
      else if (first_scan)
        scan_ts = ouster_ros::impl::ulround(*first - col_spacing_ns * first_idx);
      else
        scan_ts = ouster_ros::impl::ulround(double(LinearInterpolate(
            last_nonzero_idx, last_nonzero_ts, ts.size() + first_idx, *first, ts.size())));
      first_scan = false;

      const uint64_t *last = end - 1;
      while (*last == 0)
        --last;
      last_nonzero_idx = last - begin;
      last_nonzero_ts = *last;
      return true;
    }

    sensor_msgs::msg::PointCloud2::SharedPtr Cloud(uint64_t scan_ts)
    {
      const int h = scan.h, w = scan.w;
      const auto ts = scan.timestamp();
      const sdk::img_t<uint32_t> range = Channel(scan, sdk::ChanField::RANGE);
      const sdk::img_t<uint32_t> signal = Channel(scan, sdk::ChanField::SIGNAL);
      const sdk::img_t<uint32_t> reflectivity = Channel(scan, sdk::ChanField::REFLECTIVITY);
      const sdk::img_t<uint32_t> near_ir = Channel(scan, sdk::ChanField::NEAR_IR);
      const auto &pixel_shift_by_row = info.format.pixel_shift_by_row;
      const float nan = std::numeric_limits<float>::quiet_NaN();

      pcl::PointCloud<ouster_ros::Point> cloud(w, h);
      cloud.is_dense = true;
      for (int u = 0; u < h; ++u)
      {
        for (int v = 0; v < w; ++v)
        {
          const int v_shift = (v + w - pixel_shift_by_row[u]) % w;  // destagger
          const int src = u * w + v_shift;
          const uint32_t r = range.data()[src];
          ouster_ros::Point &pt = cloud.points[u * w + v];
          if (r <= 0 || r >= kMaxRangeMm)
          {
            pt.x = pt.y = pt.z = nan;
            cloud.is_dense = false;
          }
          else
          {
            pt.x = r * direction(src, 0) + offset(src, 0);
            pt.y = r * direction(src, 1) + offset(src, 1);
            pt.z = r * direction(src, 2) + offset(src, 2);
          }
          pt.intensity = static_cast<float>(signal.data()[src]);
          pt.t = static_cast<uint32_t>(ts[v_shift] > scan_ts ? ts[v_shift] - scan_ts : 0);
          pt.reflectivity = static_cast<uint16_t>(reflectivity.data()[src]);
          pt.ring = static_cast<uint16_t>(u);
          pt.ambient = static_cast<uint16_t>(near_ir.data()[src]);
          pt.range = r;
        }
      }

      auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      pcl::toROSMsg(cloud, *msg);
      msg->header.stamp = rclcpp::Time(scan_ts);
      msg->header.frame_id = "os_lidar";
      return msg;
    }
  };

  OusterPacketDecoder::OusterPacketDecoder(const std::string &metadata_json)
      : impl_(std::make_unique<Impl>(metadata_json)) {}

  OusterPacketDecoder::~OusterPacketDecoder() = default;

  sensor_msgs::msg::PointCloud2::SharedPtr OusterPacketDecoder::AddLidarPacket(const std::vector<uint8_t> &buf)
  {
    sdk::LidarPacket packet(static_cast<int>(buf.size()));
    packet.format = impl_->format;
    std::memcpy(packet.buf.data(), buf.data(), buf.size());
    if (impl_->first_lidar_ts == 0)
      impl_->first_lidar_ts = ouster_ros::impl::ts_safe_offset_add(
          impl_->format->col_timestamp(impl_->format->nth_col(0, packet.buf.data())), kPtpUtcTaiOffsetNs);
    if (!impl_->batcher(packet, impl_->scan))
      return nullptr;

    auto ts = impl_->scan.timestamp();  // TIME_FROM_PTP_1588: column timestamps in UTC
    for (int i = 0; i < ts.rows(); ++i)
      ts[i] = ouster_ros::impl::ts_safe_offset_add(ts[i], kPtpUtcTaiOffsetNs);
    uint64_t scan_ts;
    if (!impl_->ScanTimestamp(ts, scan_ts))
      return nullptr;
    return impl_->Cloud(scan_ts);
  }

  std::vector<sensor_msgs::msg::Imu> OusterPacketDecoder::AddImuPacket(const std::vector<uint8_t> &buf)
  {
    sdk::ImuPacket packet(static_cast<int>(buf.size()));
    packet.format = impl_->format;
    std::memcpy(packet.buf.data(), buf.data(), buf.size());
    uint64_t ts = impl_->format->imu_measurements_per_packet == 0 ? packet.gyro_ts() : packet.timestamp()[0];
    ts = ouster_ros::impl::ts_safe_offset_add(ts, kPtpUtcTaiOffsetNs);
    if (impl_->first_lidar_ts == 0)
      return {};
    auto msgs = ouster_ros::packet_to_imu_msgs(packet, "os_imu", rclcpp::Time(ts), impl_->info);
    const int64_t first_lidar_ns = impl_->first_lidar_ts;
    msgs.erase(std::remove_if(msgs.begin(), msgs.end(), [&](const sensor_msgs::msg::Imu &imu)
                              { return rclcpp::Time(imu.header.stamp).nanoseconds() < first_lidar_ns; }),
               msgs.end());
    return msgs;
  }
} // namespace cocolic

#else  // built without ouster_ros: MsgManager never creates a decoder

namespace cocolic
{
  struct OusterPacketDecoder::Impl {};
  OusterPacketDecoder::OusterPacketDecoder(const std::string &) {}
  OusterPacketDecoder::~OusterPacketDecoder() = default;
  sensor_msgs::msg::PointCloud2::SharedPtr OusterPacketDecoder::AddLidarPacket(const std::vector<uint8_t> &) { return nullptr; }
  std::vector<sensor_msgs::msg::Imu> OusterPacketDecoder::AddImuPacket(const std::vector<uint8_t> &) { return {}; }
} // namespace cocolic

#endif
