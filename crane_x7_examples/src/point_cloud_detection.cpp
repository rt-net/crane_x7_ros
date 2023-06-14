// Copyright 2023 RT Corporation
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
// https://pcl.readthedocs.io/projects/tutorials/en/master/planar_segmentation.html
// https://pcl.readthedocs.io/projects/tutorials/en/master/extract_indices.html
// https://pcl.readthedocs.io/projects/tutorials/en/master/cluster_extraction.html
//

#include <cmath>
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/common/centroid.h"
#include "pcl/common/common.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/io/pcd_io.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

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

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/classified_points", 10);

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // カメラ座標系における点群をロボット座標系に変換
    geometry_msgs::msg::TransformStamped tf_msg;

    try {
      tf_msg = tf_buffer_->lookupTransform(
        "base_link", msg->header.frame_id,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform base_link to camera_depth_optical_frame: %s",
        ex.what());
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud_transformed;
    pcl_ros::transformPointCloud("base_link", tf_msg, *msg, cloud_transformed);

    // ROSメッセージの点群フォーマットからPCLのフォーマットに変換
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::fromROSMsg(cloud_transformed, *cloud);

    // 物体検出の前処理として点群の取得範囲の制限と間引きを行う
    // フィルタリング後に点群がない場合は検出処理をスキップする
    if (preprocessing(cloud) == false) {
      return;
    }

    // 平面除去
    // 平面検知ができない場合は検出処理をスキップする
    // 物体がアームと別の高さの平面に置かれている場合など、
    // Z軸方向のフィルタリングで不要な点群が除去できない場合に使用してみてください
    /*
    if (plane_extraction(cloud) == false) {
      return;
    }
    */

    // KdTreeを用いて点群を物体ごとに分類(クラスタリング)する
    auto cluster_indices = clustering(cloud);

    // クラスタごとに色分けし、クラスタ位置をtfで配信する
    auto cloud_output = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    broadcast_cluster_position(cloud, cloud_output, cluster_indices, cloud_transformed.header);

    // クラスタリングした点群を配信する
    sensor_msgs::msg::PointCloud2 sensor_msg;
    cloud_output->header = cloud->header;
    pcl::toROSMsg(*cloud_output, sensor_msg);
    publisher_->publish(sensor_msg);
  }

  bool preprocessing(std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> & cloud)
  {
    // X軸方向0.05~0.5m以外の点群を削除
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.05, 0.5);
    pass.filter(*cloud);

    // Z軸方向0.03~0.5m以外の点群を削除
    // 物体が乗っている平面の点群を削除します
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.03, 0.5);
    pass.filter(*cloud);

    // Voxel gridで点群を間引く(ダウンサンプリング)
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud);

    // フィルタリング後に点群がない場合はfalseを返す
    if (cloud->size() <= 0) {
      RCLCPP_INFO(this->get_logger(), "No point cloud in the detection area.");
      return false;
    } else {
      return true;
    }
  }

  bool plane_extraction(std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> & cloud)
  {
    auto coefficients = std::make_shared<pcl::ModelCoefficients>();
    auto inliers = std::make_shared<pcl::PointIndices>();
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    // 平面が検出できなかった場合
    if (inliers->indices.size() <= 0) {
      RCLCPP_INFO(this->get_logger(), "Could not estimate a planar model for the given dataset.");
      return false;
    }

    // 検出した平面を削除
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud);
    return true;
  }

  std::vector<pcl::PointIndices> clustering(
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> & cloud)
  {
    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(250);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    return cluster_indices;
  }

  void broadcast_cluster_position(
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> & cloud_input,
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> & cloud_output,
    std::vector<pcl::PointIndices> & cluster_indices,
    std_msgs::msg::Header & tf_header)
  {
    int cluster_i = 0;
    enum COLOR_RGB
    {
      RED = 0,
      GREEN,
      BLUE,
      COLOR_MAX
    };
    const int CLUSTER_MAX = 10;
    const int CLUSTER_COLOR[CLUSTER_MAX][COLOR_MAX] = {
      {230, 0, 18}, {243, 152, 18}, {255, 251, 0},
      {143, 195, 31}, {0, 153, 68}, {0, 158, 150},
      {0, 160, 233}, {0, 104, 183}, {29, 32, 136},
      {146, 7, 131}
    };

    for (const auto & point_indices : cluster_indices) {
      auto cloud_cluster = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
      // 点群の色を変更
      for (const auto & point_i : point_indices.indices) {
        cloud_input->points[point_i].r = CLUSTER_COLOR[cluster_i][RED];
        cloud_input->points[point_i].g = CLUSTER_COLOR[cluster_i][GREEN];
        cloud_input->points[point_i].b = CLUSTER_COLOR[cluster_i][BLUE];
        cloud_cluster->points.push_back(cloud_input->points[point_i]);
      }
      // 点群数の入力
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      // 無効なpointがないのでis_denseはtrue
      cloud_cluster->is_dense = true;
      // 配信用の点群に追加
      *cloud_output += *cloud_cluster;

      // tfの配信
      // 点群の重心位置を物体位置として配信する
      Eigen::Vector4f cluster_centroid;
      pcl::compute3DCentroid(*cloud_cluster, cluster_centroid);
      geometry_msgs::msg::TransformStamped t;
      t.header = tf_header;
      t.child_frame_id = "target_" + std::to_string(cluster_i);
      t.transform.translation.x = cluster_centroid.x();
      t.transform.translation.y = cluster_centroid.y();
      t.transform.translation.z = cluster_centroid.z();
      t.transform.rotation.x = 0.0;
      t.transform.rotation.y = 0.0;
      t.transform.rotation.z = 0.0;
      t.transform.rotation.w = 1.0;
      tf_broadcaster_->sendTransform(t);

      // 設定した最大クラスタ数を超えたら処理を終える
      cluster_i++;
      if (cluster_i >= CLUSTER_MAX) {
        break;
      }
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudSubscriber>());
  rclcpp::shutdown();
  return 0;
}
