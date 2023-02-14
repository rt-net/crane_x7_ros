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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(cloud_transformed, *cloud);

    // 取得した点群すべてを認識対象にすると処理が重いため前処理(取得範囲の制限や間引き)を行う
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    // X軸方向0.05~0.5m以外の点群を削除
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.05, 0.4);
    pass.filter(*cloud_filtered);

    // Z軸方向0.03~0.5m以外の点群を削除
    // 物体が乗っている平面の点群を削除します
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.03, 0.5);
    pass.filter(*cloud_filtered);

    // Voxel gridで点群を間引く(ダウンサンプリング)
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud_filtered);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered);

    // 点群がない場合は物体認識処理をスキップする
    if (cloud_filtered->size() <= 0) {
      return;
    }

    // 平面検出
    // 物体がアームと別の高さの平面に置かれている場合など、
    // Z軸方向のフィルタリングで不要な点群が除去できない場合に使用してみてください
    /*
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);

    // 平面検出に失敗した場合は物体認識をスキップする
    if (inliers->indices.size() == 0) {
      RCLCPP_INFO(this->get_logger(), "Could not estimate a planar model for the given dataset.");
      return;
    }

    // 検出した平面を削除
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);
    */

    // KdTreeを用いて点群を物体ごとに分類(クラスタリング)する
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(250);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    // クラスタリングした点群ごとに異なる色をつける
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZRGB>());
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

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
      it != cluster_indices.end(); ++it)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>());
      // 点群の色を変更
      for (std::vector<int>::const_iterator pit = it->indices.begin();
        pit != it->indices.end(); ++pit)
      {
        cloud_filtered->points[*pit].r = CLUSTER_COLOR[cluster_i][RED];
        cloud_filtered->points[*pit].g = CLUSTER_COLOR[cluster_i][GREEN];
        cloud_filtered->points[*pit].b = CLUSTER_COLOR[cluster_i][BLUE];
        cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
      }
      // 点群サイズの入力
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
      t.header = cloud_transformed.header;
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

    // クラスタリングした点群を配信する
    sensor_msgs::msg::PointCloud2 sensor_msg;
    cloud_output->header = cloud->header;
    pcl::toROSMsg(*cloud_output, sensor_msg);
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
