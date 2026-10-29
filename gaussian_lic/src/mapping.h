/*
 * Gaussian-LIC: Real-Time Photo-Realistic SLAM with Gaussian Splatting and LiDAR-Inertial-Camera Fusion
 * Copyright (C) 2025 Xiaolei Lang
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

#pragma once

#include "yaml_utils.h"

#include <chrono>
#include <deque>
#include <queue>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Eigen>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

class Params
{
public:
    Params(const YAML::Node &node)
    {
        height = node["height"].as<int>();
        width = node["width"].as<int>();
        fx = node["fx"].as<double>();
        fy = node["fy"].as<double>();
        cx = node["cx"].as<double>();
        cy = node["cy"].as<double>();

        select_every_k_frame = node["select_every_k_frame"].as<int>();
        depth_completion = node["depth_completion"].as<bool>();
        patch_size = node["patch_size"].as<int>();
        max_depth = node["max_depth"].as<double>();
        std::string pkg_path = ament_index_cpp::get_package_share_directory("gaussian_lic");
        // depth completion engine for this image size, built by ckpt/build_trt.sh
        engine_path = pkg_path + "/ckpt/spnet_" + std::to_string(height) + "_" + std::to_string(width) + ".engine";

        sh_degree = node["sh_degree"].as<int>();
        white_background = node["white_background"].as<bool>();
        random_background = node["random_background"].as<bool>();
        convert_SHs_python = node["convert_SHs_python"].as<bool>();
        compute_cov3D_python = node["compute_cov3D_python"].as<bool>();
        lambda_erank = node["lambda_erank"].as<double>();
        scaling_scale = node["scaling_scale"].as<double>();

        position_lr = node["position_lr"].as<double>();
        feature_lr = node["feature_lr"].as<double>();
        opacity_lr = node["opacity_lr"].as<double>();
        scaling_lr = node["scaling_lr"].as<double>();
        rotation_lr = node["rotation_lr"].as<double>();
        lambda_dssim = node["lambda_dssim"].as<double>();
        optimize_depth = node["optimize_depth"].as<bool>();
        lambda_depth = node["lambda_depth"].as<double>();
        iteration_decay = node["iteration_decay"].as<bool>();

        apply_exposure = node["apply_exposure"].as<bool>();
        exposure_lr = node["exposure_lr"].as<double>();
        skybox_points_num = node["skybox_points_num"].as<int>();
        skybox_radius = node["skybox_radius"].as<int>();
    }

    /// dataset
    int height;
    int width;
    double fx;
    double fy;
    double cx;
    double cy;

    int select_every_k_frame;
    bool depth_completion;
    int patch_size;
    double max_depth;
    std::string engine_path;

    /// gaussian
    int sh_degree;
    bool white_background;
    bool random_background;
    bool convert_SHs_python;
    bool compute_cov3D_python;
    float lambda_erank;
    double scaling_scale;

    double position_lr;
    double feature_lr;
    double opacity_lr;
    double scaling_lr;
    double rotation_lr;
    double lambda_dssim;
    bool optimize_depth;
    double lambda_depth;
    bool iteration_decay;

    bool apply_exposure;
    double exposure_lr;
    int skybox_points_num;
    int skybox_radius;
};

struct Frame 
{
    sensor_msgs::msg::PointCloud2::ConstSharedPtr point_msg;
    geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg;
    sensor_msgs::msg::Image::ConstSharedPtr image_msg;
    sensor_msgs::msg::Image::ConstSharedPtr depth_msg;
};