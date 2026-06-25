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

#include <memory>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "image_transport/camera_subscriber.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/core/quaternion.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;
using std::placeholders::_2;

class ArucoDetector : public rclcpp::Node
{
public:
  ArucoDetector()
  : Node("aruco_detection")
  {
    // カメラ画像とカメラ情報のトピックを同期して受信するためのサブスクライバ
    camera_subscription_ = image_transport::create_camera_subscription(
      this, "/camera/color/image_raw", std::bind(&ArucoDetector::camera_callback, this, _1, _2),
      "raw");

    // ArUcoマーカのデータセットを読み込む
    // DICT_6x6_50は6x6ビットのマーカが50個収録されたもの
    marker_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);

    // 検出したマーカの位置姿勢を配信するためのTransformBroadcasterを初期化
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  image_transport::CameraSubscriber camera_subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  cv::Ptr<cv::aruco::Dictionary> marker_dict_;

  // カメラ情報を受信した時に動作するコールバック
  void camera_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & img_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
  {
    // ROSの画像トピックからOpenCVで扱える画像に変換
    auto cv_img = cv_bridge::toCvShare(img_msg, img_msg->encoding);
    cv::cvtColor(cv_img->image, cv_img->image, cv::COLOR_RGB2BGR);

    // マーカIDの格納配列
    std::vector<int> ids;
    // 画像座標系上のマーカ頂点位置の格納配列
    std::vector<std::vector<cv::Point2f>> corners;
    // 画像からマーカを検出
    cv::aruco::detectMarkers(cv_img->image, marker_dict_, corners, ids);
    // 検出されたマーカの個数
    int n_markers = ids.size();

    if (n_markers <= 0) {
      return;
    }

    // カメラのキャリブレーションパラメータ（内部パラメータ、歪みパラメータ）をロード
    const auto CAMERA_MATRIX = cv::Mat(3, 3, CV_64F, const_cast<double *>(info_msg->k.data()));
    const auto DIST_COEFFS = cv::Mat(1, 5, CV_64F, const_cast<double *>(info_msg->d.data()));
    // 使用している実物マーカ一辺の長さ 0.04 [m]
    const float MARKER_LENGTH = 0.04;
    // 検出した各マーカの回転ベクトルと平行移動ベクトル
    std::vector<cv::Vec3d> rvecs, tvecs;
    // 画像（2D）のマーカの角からカメラ座標系に対する三次元の位置姿勢を推定
    cv::aruco::estimatePoseSingleMarkers(
      corners, MARKER_LENGTH, CAMERA_MATRIX, DIST_COEFFS, rvecs, tvecs);

    // 検出したすべてのマーカを個別にTFとして配信
    for (int i = 0; i < n_markers; i++) {
      geometry_msgs::msg::TransformStamped t;
      t.header = img_msg->header;
      t.child_frame_id = "target_" + std::to_string(ids[i]);
      t.transform.translation.x = tvecs[i][0];
      t.transform.translation.y = tvecs[i][1];
      t.transform.translation.z = tvecs[i][2];
      // 回転ベクトルをクォータニオンへ変換
      cv::Quatd cv_q = cv::Quatd::createFromRvec(rvecs[i]);
      t.transform.rotation.x = cv_q.x;
      t.transform.rotation.y = cv_q.y;
      t.transform.rotation.z = cv_q.z;
      t.transform.rotation.w = cv_q.w;
      tf_broadcaster_->sendTransform(t);
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // ArucoDetectorノードを実行
  rclcpp::spin(std::make_shared<ArucoDetector>());
  rclcpp::shutdown();
  return 0;
}
