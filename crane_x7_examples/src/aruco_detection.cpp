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
// https://docs.opencv.org/4.2.0/d5/dae/tutorial_aruco_detection.html
// https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Cpp.html

#include <cmath>
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
using std::placeholders::_1;

class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber()
  : Node("aruco_detection")
  {
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", 10, std::bind(&ImageSubscriber::image_callback, this, _1));

    camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/color/camera_info", 10, std::bind(&ImageSubscriber::camera_info_callback, this, _1));

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    auto cv_img = cv_bridge::toCvShare(msg, msg->encoding);
    cv::cvtColor(cv_img->image, cv_img->image, cv::COLOR_RGB2BGR);

    if (camera_info_) {
      // ArUcoマーカのデータセットを読み込む
      // DICT_6x6_50は6x6ビットのマーカが50個収録されたもの
      const auto MARKER_DICT = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
      // マーカID
      std::vector<int> ids;
      // 画像座標系上のマーカ頂点位置
      std::vector<std::vector<cv::Point2f>> corners;
      // マーカの検出
      cv::aruco::detectMarkers(cv_img->image, MARKER_DICT, corners, ids);
      // マーカの検出数
      int n_markers = ids.size();
      // カメラパラメータ
      const auto CAMERA_MATRIX = cv::Mat(3, 3, CV_64F, camera_info_->k.data());
      const auto DIST_COEFFS = cv::Mat(1, 5, CV_64F, camera_info_->d.data());
      // マーカ一辺の長さ 0.04 [m]
      const float MARKER_LENGTH = 0.04;

      // マーカが一つ以上検出された場合、マーカの位置姿勢をtfで配信
      if (n_markers > 0) {
        for (int i = 0; i < n_markers; i++) {
          // マーカの回転ベクトルと位置ベクトル
          std::vector<cv::Vec3d> rvecs, tvecs;
          // 画像座標系上のマーカ位置を三次元のカメラ座標系に変換
          cv::aruco::estimatePoseSingleMarkers(
            corners, MARKER_LENGTH, CAMERA_MATRIX, DIST_COEFFS, rvecs, tvecs);

          // tfの配信
          geometry_msgs::msg::TransformStamped t;
          t.header.stamp = this->get_clock()->now();
          t.header.frame_id = "camera_color_optical_frame";
          t.child_frame_id = "target_" + std::to_string(ids[i]);
          t.transform.translation.x = tvecs[i][0];
          t.transform.translation.y = tvecs[i][1];
          t.transform.translation.z = tvecs[i][2];
          tf2::Quaternion q;
          cv::Mat cv_rotation_matrix;
          cv::Rodrigues(rvecs[i], cv_rotation_matrix);
          tf2::Matrix3x3 tf2_rotation_matrix = tf2::Matrix3x3(
            cv_rotation_matrix.at<double>(0, 0),
            cv_rotation_matrix.at<double>(0, 1),
            cv_rotation_matrix.at<double>(0, 2),
            cv_rotation_matrix.at<double>(1, 0),
            cv_rotation_matrix.at<double>(1, 1),
            cv_rotation_matrix.at<double>(1, 2),
            cv_rotation_matrix.at<double>(2, 0),
            cv_rotation_matrix.at<double>(2, 1),
            cv_rotation_matrix.at<double>(2, 2));
          tf2_rotation_matrix.getRotation(q);
          t.transform.rotation.x = q.x();
          t.transform.rotation.y = q.y();
          t.transform.rotation.z = q.z();
          t.transform.rotation.w = q.w();
          tf_broadcaster_->sendTransform(t);
        }
      }
    }
  }

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    camera_info_ = msg;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}
