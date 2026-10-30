/*
 * Coco-LIC: Continuous-Time Tightly-Coupled LiDAR-Inertial-Camera Odometry using Non-Uniform B-spline
 * Copyright (C) 2023 Xiaolei Lang
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <rclcpp/rclcpp.hpp>

#include <odom/odometry_manager.h>

using namespace cocolic;

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);

  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr nh = std::make_shared<rclcpp::Node>("cocolic");

  std::string config_path = nh->declare_parameter<std::string>("config_path", "ct_odometry.yaml");
  RCLCPP_INFO(nh->get_logger(), "Odometry load %s.", config_path.c_str());

  YAML::Node config_node = YAML::LoadFile(config_path);
  // another camera config for the same setup, e.g. the payload's camera preview (payload.launch.xml camera:=)
  std::string camera_yaml = nh->declare_parameter<std::string>("camera_yaml", "");
  if (!camera_yaml.empty())
    config_node["camera_yaml"] = camera_yaml;

  // std::string log_path = config_node["log_path"].as<std::string>();
  // FLAGS_log_dir = log_path;
  // FLAGS_colorlogtostderr = true;
  std::cout << "\n🥥 Start Coco-LIC Odometry 🥥";

  OdometryManager odom_manager(config_node, nh);
  odom_manager.RunBag();

  if (odom_manager.IsInitialized())
  {
    double t_traj_max = odom_manager.SaveOdometry();
    if (rclcpp::ok())  // after Ctrl-C, Gaussian-LIC takes no more data
      odom_manager.WaitForPublishedData();
  }
  else
  {
    std::cout << "\n🍺 Stopped before the initialization: no trajectory to save.\n";
  }
  std::cout << "\n✨ All Done.\n\n";

  rclcpp::shutdown();
  return 0;
}
