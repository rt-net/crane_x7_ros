// Copyright 2022 RT Corporation
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

// Reference:
// https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Cpp.html
// https://pcl.readthedocs.io/projects/tutorials/en/master/passthrough.html
// https://pcl.readthedocs.io/projects/tutorials/en/master/voxel_grid.html
// 

#include <cmath>
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/common/common.h"
#include "pcl/io/pcd_io.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl_ros/transforms.hpp"

class PointCloudSubscriber : public rclcpp::Node
{
public:
  PointCloudSubscriber()
  : Node("point_cloud_detection")
  {
    point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/depth/color/points",
      10,
      std::bind(&PointCloudSubscriber::point_cloud_callback, this, std::placeholders::_1));

    using namespace std::chrono_literals;
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcl_data", 10);

    //tf_broadcaster_ =
    //  std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  //std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // ID 0のマーカ位置姿勢を取得
    geometry_msgs::msg::TransformStamped tf_msg;

    try {
      tf_msg = tf_buffer_->lookupTransform(
        "base_link", msg->header.frame_id,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform base_link to target: %s",
        ex.what());
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud_transformed;
    pcl_ros::transformPointCloud("base_link", tf_msg, *msg, cloud_transformed);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(cloud_transformed, *cloud);

    // X軸方向の0.05~0.5m内の点群を使用
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.05, 0.5);
    pass.filter(*cloud_filtered);

    // Z軸方向の0.03~0.5m内の点群を使用
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.03, 0.5);
    pass.filter(*cloud_filtered);

    // Voxel gridでダウンサンプリング
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud_filtered);
    sor.setLeafSize(0.005f, 0.005f, 0.005f);
    sor.filter(*cloud_filtered);

    sensor_msgs::msg::PointCloud2 sensor_msg;
    pcl::toROSMsg(*cloud_filtered, sensor_msg);
    publisher_->publish(sensor_msg);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudSubscriber>());
  rclcpp::shutdown();
  return 0;
}
