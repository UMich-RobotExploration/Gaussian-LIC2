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

#include "mapping.h"
#include "gaussian.h"

#include <atomic>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

std::mutex m_buf;
std::condition_variable con;

std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> point_buf;
std::queue<geometry_msgs::msg::PoseStamped::ConstSharedPtr> pose_buf;
std::queue<sensor_msgs::msg::Image::ConstSharedPtr> image_buf;
std::queue<sensor_msgs::msg::Image::ConstSharedPtr> depth_buf;

std::atomic<bool> exit_flag(false);
std::atomic<double> last_point_time(0.0);
std::atomic<bool> gaussians_initialized(false);

inline double toSec(const builtin_interfaces::msg::Time& stamp)
{
    return rclcpp::Time(stamp).seconds();
}

// Steady (wall) clock for the end-of-data timeout: with use_sim_time, ROS time freezes
// once the bag stops publishing /clock, so the timeout would never fire.
inline double steadyNow()
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void pointCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& point_msg)
{
    m_buf.lock();
    point_buf.push(point_msg);
    last_point_time = steadyNow();
    m_buf.unlock();
}

void poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_msg) 
{
    m_buf.lock();
    pose_buf.push(pose_msg);
    m_buf.unlock();
}

void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg) 
{
    m_buf.lock();
    image_buf.push(image_msg);
    m_buf.unlock();
}

void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) 
{
    m_buf.lock();
    depth_buf.push(depth_msg);
    m_buf.unlock();
}

bool getAlignedData(Frame& cur_frame)
{
    if (point_buf.empty() || pose_buf.empty() || image_buf.empty() || depth_buf.empty()) 
    {
        return false;
    }

    double frame_time = toSec(point_buf.front()->header.stamp);

    while (1) 
    {
        if (toSec(pose_buf.front()->header.stamp) < frame_time - 0.01) 
        {
            pose_buf.pop();
            if (pose_buf.empty()) 
            {
                return false;
            }
        } 
        else break;
    }
    if (toSec(pose_buf.front()->header.stamp) > frame_time + 0.01) 
    {
        point_buf.pop();
        return false;
    }

    while (1) 
    {
        if (toSec(image_buf.front()->header.stamp) < frame_time - 0.01) 
        {
            image_buf.pop();
            if (image_buf.empty()) 
            {
                return false;
            }
        } 
        else break;
    }
    if (toSec(image_buf.front()->header.stamp) > frame_time + 0.01) 
    {
        point_buf.pop();
        return false;
    }

    while (1) 
    {
        if (toSec(depth_buf.front()->header.stamp) < frame_time - 0.01) 
        {
            depth_buf.pop();
            if (depth_buf.empty()) 
            {
                return false;
            }
        } 
        else break;
    }
    if (toSec(depth_buf.front()->header.stamp) > frame_time + 0.01) 
    {
        point_buf.pop();
        return false;
    }

    auto cur_point = point_buf.front();
    auto cur_pose = pose_buf.front();
    auto cur_image = image_buf.front();
    auto cur_depth = depth_buf.front();

    cur_frame.point_msg = cur_point;
    cur_frame.pose_msg = cur_pose;
    cur_frame.image_msg = cur_image;
    cur_frame.depth_msg = cur_depth;

    point_buf.pop();
    pose_buf.pop();
    image_buf.pop();
    depth_buf.pop();

    return true;
}

// RViz2 visualization of the Gaussian map, published only while someone subscribes
struct MapViz
{
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map;  // Gaussian centres in their base (SH degree 0) colour
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr render;     // splat render from the latest keyframe camera
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr splats;  // all Gaussian parameters, for the 3D splat viewer
    double last_map_time = -1;
    double last_splat_time = -1;
};

void publishGaussianMap(MapViz& viz, const std::shared_ptr<GaussianModel>& pc)
{
    if (viz.map->get_subscription_count() == 0) return;
    torch::NoGradGuard no_grad;
    torch::Tensor keep = pc->getOpacity().reshape({-1}) > 0.1f;  // hide nearly transparent Gaussians
    torch::Tensor xyz = pc->getXYZ().index({keep}).to(torch::kCPU).contiguous();
    torch::Tensor rgb = (pc->getFeaturesDc().index({keep}).reshape({-1, 3}) * C0 + 0.5).clamp(0, 1)
                            .mul(255).to(torch::kU8).to(torch::kCPU).contiguous();

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud.resize(xyz.size(0));
    auto xyz_a = xyz.accessor<float, 2>();
    auto rgb_a = rgb.accessor<uint8_t, 2>();
    for (int64_t i = 0; i < xyz.size(0); ++i)
    {
        cloud[i].x = xyz_a[i][0];
        cloud[i].y = xyz_a[i][1];
        cloud[i].z = xyz_a[i][2];
        cloud[i].r = rgb_a[i][0];
        cloud[i].g = rgb_a[i][1];
        cloud[i].b = rgb_a[i][2];
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = viz.node->now();
    viz.map->publish(msg);
}

// one point per Gaussian with the values the rasterizer uses: base colour, sigmoid(opacity), exp(scaling),
// normalized rotation quaternion (w, x, y, z)
void publishSplats(MapViz& viz, const std::shared_ptr<GaussianModel>& pc)
{
    if (viz.splats->get_subscription_count() == 0) return;
    torch::NoGradGuard no_grad;
    torch::Tensor rgb = (pc->getFeaturesDc().reshape({-1, 3}) * C0 + 0.5).clamp(0, 1);
    torch::Tensor data = torch::cat({pc->getXYZ(), rgb, pc->getOpacity().reshape({-1, 1}), pc->getScaling(), pc->getRotation()}, 1)
                             .to(torch::kFloat32).to(torch::kCPU).contiguous();

    static const std::vector<std::string> names = {"x", "y", "z", "red", "green", "blue", "opacity",
                                                   "scale_0", "scale_1", "scale_2", "rot_0", "rot_1", "rot_2", "rot_3"};
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = "map";
    msg.header.stamp = viz.node->now();
    msg.height = 1;
    msg.width = data.size(0);
    for (size_t i = 0; i < names.size(); ++i)
    {
        sensor_msgs::msg::PointField field;
        field.name = names[i];
        field.offset = 4 * i;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        msg.fields.push_back(field);
    }
    msg.is_bigendian = false;
    msg.point_step = 4 * names.size();
    msg.row_step = msg.point_step * msg.width;
    msg.is_dense = true;
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(data.data_ptr<float>());
    msg.data.assign(bytes, bytes + data.numel() * sizeof(float));
    viz.splats->publish(msg);
}

void publishRender(MapViz& viz, const std::shared_ptr<Dataset>& dataset, std::shared_ptr<GaussianModel>& pc)
{
    if (viz.render->get_subscription_count() == 0 || dataset->train_cameras_.empty()) return;
    torch::NoGradGuard no_grad;
    torch::Tensor bg = pc->white_background_ ? torch::ones({3}, torch::kFloat32).cuda() : torch::zeros({3}, torch::kFloat32).cuda();
    torch::Tensor image = std::get<0>(render(dataset->train_cameras_.back(), pc, bg, pc->apply_exposure_));
    image = image.clamp(0, 1).mul(255).to(torch::kU8).permute({1, 2, 0}).to(torch::kCPU).contiguous();

    sensor_msgs::msg::Image msg;
    msg.header.frame_id = "camera";
    msg.header.stamp = viz.node->now();
    msg.height = image.size(0);
    msg.width = image.size(1);
    msg.encoding = "rgb8";
    msg.step = msg.width * 3;
    msg.data.assign(image.data_ptr<uint8_t>(), image.data_ptr<uint8_t>() + image.numel());
    viz.render->publish(msg);
}

void mapping(const YAML::Node& node, const std::string& result_path, const std::string& lpips_path, MapViz& viz)
{
    torch::jit::setGraphExecutorOptimize(false);

    Params prm(node);
    std::shared_ptr<GaussianModel> gaussians = std::make_shared<GaussianModel>(prm);
    std::shared_ptr<Dataset> dataset = std::make_shared<Dataset>(prm);

    std::chrono::steady_clock::time_point t_start, t_end;
    double total_mapping_time = 0;
    double total_adding_time = 0;
    double total_extending_time = 0;

    Frame cur_frame;
    while (!exit_flag)
    {
        /// [1] data alignment
        m_buf.lock();
        bool align_flag = getAlignedData(cur_frame);
        m_buf.unlock();
        if (!align_flag) continue;
        
        /// [2] add every frame
        t_start = std::chrono::steady_clock::now();
        dataset->addFrame(cur_frame);
        torch::cuda::synchronize();
        t_end = std::chrono::steady_clock::now();
        if (dataset->is_keyframe_current_)
        {
            total_adding_time += std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start).count();
            std::cout << "\033[1;33m     Cur Frame " << dataset->all_frame_num_ - 1 << ",\033[0m";
        }
        else continue;

        if (!gaussians->is_init_)
        {
            /// [3] initialize map
            gaussians->is_init_ = true;
            gaussians_initialized = true;
            gaussians->initialize(dataset);
            gaussians->trainingSetup();
        }
        else 
        {
            /// [4] extend map
            t_start = std::chrono::steady_clock::now();
            extend(dataset, gaussians);
            torch::cuda::synchronize();
            t_end = std::chrono::steady_clock::now();
            total_extending_time += std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start).count();
        }

        /// [5] optimize map
        t_start = std::chrono::steady_clock::now();
        double updated_num = optimize(dataset, gaussians);
        torch::cuda::synchronize();
        t_end = std::chrono::steady_clock::now();
        total_mapping_time += std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start).count();
        std::cout << std::fixed << std::setprecision(2) 
                  << "\033[1;36m Update " << updated_num / 10000 
                  << "w GS per Iter \033[0m" << std::endl;

        /// visualize in RViz2
        publishRender(viz, dataset, gaussians);
        if (steadyNow() - viz.last_map_time > 1.0)
        {
            publishGaussianMap(viz, gaussians);
            viz.last_map_time = steadyNow();
        }
        if (steadyNow() - viz.last_splat_time > 2.0)
        {
            publishSplats(viz, gaussians);
            viz.last_splat_time = steadyNow();
        }
    }

    if (!gaussians->is_init_)  // stopped before the first keyframe
    {
        std::cout << "\n\n😋 Gaussian-LIC got no keyframe: no map to save.\n\n\n";
        rclcpp::shutdown();
        return;
    }

    /// [6] evaluation
    std::cout << "\n     🎉 Runtime Statistics 🎉\n";
    std::cout << std::fixed << std::setprecision(2) << "\n        [Total Mapping Time] " << total_mapping_time << "s" << std::endl;
    std::cout << std::fixed << std::setprecision(2) << "         1) Forward " << gaussians->t_forward_ << "s" << std::endl;
    std::cout << std::fixed << std::setprecision(2) << "         2) Backward " << gaussians->t_backward_ << "s" << std::endl;
    std::cout << std::fixed << std::setprecision(2) << "         3) Step " << gaussians->t_step_ << "s" << std::endl;
    std::cout << std::fixed << std::setprecision(2) << "         4) CPU2GPU " << gaussians->t_tocuda_ << "s" << std::endl;
    std::cout << std::fixed << std::setprecision(2) << "        [Total Adding Time] " << total_adding_time << "s" << std::endl;
    std::cout << std::fixed << std::setprecision(2) << "        [Total Extending Time] " << total_extending_time << "s" << std::endl;
    torch::NoGradGuard no_grad;
    evaluateVisualQuality(dataset, gaussians, result_path, lpips_path);
    gaussians->saveMap(result_path);
    publishGaussianMap(viz, gaussians);
    publishSplats(viz, gaussians);
    if (rclcpp::ok())  // after Ctrl-C nothing is delivered any more
    {
        viz.map->wait_for_all_acked(std::chrono::seconds(5));
        viz.splats->wait_for_all_acked(std::chrono::seconds(10));
    }

    std::cout << "\n\n😋 Gaussian-LIC Done!\n\n\n";
    rclcpp::shutdown();  // results are saved: stop spinning so the node exits on its own
}

int main(int argc, char** argv)
{
    std::cout << "\n\n😋 Gaussian-LIC Ready!\n\n\n";
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr nh = std::make_shared<rclcpp::Node>("gaussianlic");
    image_transport::ImageTransport it_(nh);

    auto sub_point = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/points_for_gs", 10000, pointCallback);
    auto sub_pose = nh->create_subscription<geometry_msgs::msg::PoseStamped>("/pose_for_gs", 10000, poseCallback);
    image_transport::Subscriber image_sub = it_.subscribe("/image_for_gs", 10000, imageCallback);
    image_transport::Subscriber depth_sub = it_.subscribe("/depth_for_gs", 10000, depthCallback);

    std::string config_path = nh->declare_parameter<std::string>("config_path", "");
    YAML::Node config_node = YAML::LoadFile(config_path);
    std::string result_path = nh->declare_parameter<std::string>("result_path", "");
    std::string lpips_path = nh->declare_parameter<std::string>("lpips_path", "");

    MapViz viz;
    viz.node = nh;
    viz.map = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/gaussian_lic/map", rclcpp::QoS(1).transient_local());
    viz.render = nh->create_publisher<sensor_msgs::msg::Image>("/gaussian_lic/render", 10);
    viz.splats = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/gaussian_lic/splats", rclcpp::QoS(1).transient_local());

    std::thread mapping_process(mapping, config_node, result_path, lpips_path, std::ref(viz));
    std::thread monitor_thread([](){
        while (!exit_flag) 
        {
            double now = steadyNow();
            if (gaussians_initialized && (now - last_point_time > 5.0))
            {
                m_buf.lock();
                if (point_buf.empty()) exit_flag = true;
                m_buf.unlock();
            }
            else if (!gaussians_initialized && !rclcpp::ok())  // Ctrl-C before the first keyframe
            {
                exit_flag = true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    });
    
    rclcpp::spin(nh);

    mapping_process.join();
    monitor_thread.join();
    
    rclcpp::shutdown();
    return 0;
}