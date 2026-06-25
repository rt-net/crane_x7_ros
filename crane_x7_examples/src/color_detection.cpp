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
// https://www.opencv-srf.com/2010/09/object-detection-using-color-seperation.html

#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "image_geometry/pinhole_camera_model.hpp"
#include "image_transport/image_transport.hpp"
#include "image_transport/subscriber_filter.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class ColorDetector : public rclcpp::Node
{
public:
  ColorDetector()
  : Node("color_detection")
  {
    // 各トピックのサブスクライバを初期化
    color_sub_.subscribe(this, "/camera/color/image_raw", "raw");
    depth_sub_.subscribe(this, "/camera/aligned_depth_to_color/image_raw", "raw");
    info_sub_.subscribe(this, "/camera/color/camera_info");

    // カラー画像、深度マップ、カメラパラメータのメッセージを完全に一致するタイムスタンプで同期して受信する
    sync_ = std::make_unique<message_filters::Synchronizer<ExactPolicy>>(
      ExactPolicy(10), color_sub_, depth_sub_, info_sub_);
    sync_->registerCallback(std::bind(&ColorDetector::sync_callback, this, _1, _2, _3));

    // 二値化処理した確認用の画像をパブリッシュするためのパブリッシャ
    image_thresholded_publisher_ =
      this->create_publisher<sensor_msgs::msg::Image>("image_thresholded", 10);

    // 検出した位置をtarget_0としてTFフレームに流すブロードキャスタ
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  using ExactPolicy = message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
  image_transport::SubscriberFilter color_sub_;
  image_transport::SubscriberFilter depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
  std::unique_ptr<message_filters::Synchronizer<ExactPolicy>> sync_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_thresholded_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // 同期されたメッセージを受信したときに呼び出されるコールバック
  void sync_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & color_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
  {
    // 青い物体を検出するようにHSVの抽出範囲を設定
    // 周囲の明るさ等の環境や、検出させたい色によって微調整します
    const int LOW_H = 100, HIGH_H = 125;
    const int LOW_S = 100, HIGH_S = 255;
    const int LOW_V = 30, HIGH_V = 255;

    // ROSカラー画像トピックをOpenCVで扱える画像に変換
    auto cv_color = cv_bridge::toCvShare(color_msg, color_msg->encoding);

    // HSVカラースペースに画像フォーマットを変換
    cv::cvtColor(cv_color->image, cv_color->image, cv::COLOR_RGB2HSV);

    // 画像処理結果用バッファ
    cv::Mat img_thresholded;

    // 設定範囲でHSV成分を二値化抽出（指定の青い部分だけが白（255）、他は黒（0）になります）
    cv::inRange(
      cv_color->image, cv::Scalar(LOW_H, LOW_S, LOW_V), cv::Scalar(HIGH_H, HIGH_S, HIGH_V),
      img_thresholded);

    // モルフォロジー演算（オープニング処理）で孤立点などのゴミを除去
    cv::morphologyEx(
      img_thresholded, img_thresholded, cv::MORPH_OPEN,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

    // モルフォロジー演算（クロージング処理）で青色の領域内の穴を埋める
    cv::morphologyEx(
      img_thresholded, img_thresholded, cv::MORPH_CLOSE,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

    // 検出した白い領域全体のモーメント（重心）を計算
    cv::Moments moment = moments(img_thresholded);
    double d_m01 = moment.m01;
    double d_m10 = moment.m10;
    double d_area = moment.m00;  // 面積に相当

    // 検出した領域があまりにも小さい場合（面積が10000ピクセル以下）はノイズと見なして無視
    if (d_area <= 10000) {
      return;
    }

    // カメラのキャリブレーション内部パラメータ情報を基にPinholeCameraModelオブジェクトを作成
    image_geometry::PinholeCameraModel camera_model;
    camera_model.fromCameraInfo(*info_msg);

    // 画像面（2D）における青いオブジェクトの中心重心座標を計算
    const double pixel_x = d_m10 / d_area;
    const double pixel_y = d_m01 / d_area;
    const cv::Point2d point(pixel_x, pixel_y);

    // カメラ歪みを補正した後の2D画像座標を算出
    const cv::Point2d rect_point = camera_model.rectifyPoint(point);

    // カメラ焦点の中心からその2D点への3D方向ベクトル（Ray）を算出
    const cv::Point3d ray = camera_model.projectPixelTo3dRay(rect_point);

    // オブジェクト表面から把持したい深さへのオフセット値
    const double DEPTH_OFFSET = 0.015;
    auto cv_depth = cv_bridge::toCvShare(depth_msg, depth_msg->encoding);

    // 深度情報（mm単位）から、カメラからオブジェクト表面までの直線的な奥行き（m）を算出
    const auto front_distance = cv_depth->image.at<ushort>(point) / 1000.0;
    const auto center_distance = front_distance + DEPTH_OFFSET;

    // 測定範囲限界の設定（近すぎず、かつロボットの可動限界である50cm以内であるか）
    const double DEPTH_MAX = 0.5;
    const double DEPTH_MIN = 0.2;
    if (center_distance < DEPTH_MIN || center_distance > DEPTH_MAX) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Failed to get depth at " << point << ".");
      return;
    }

    // オブジェクトの3次元位置姿勢を計算
    cv::Point3d object_position(
      ray.x * center_distance, ray.y * center_distance, ray.z * center_distance);

    // 3D把持対象位置をtarget_0という名でTF配信する
    geometry_msgs::msg::TransformStamped t;
    t.header = color_msg->header;
    t.child_frame_id = "target_0";
    t.transform.translation.x = object_position.x;
    t.transform.translation.y = object_position.y;
    t.transform.translation.z = object_position.z;
    tf_broadcaster_->sendTransform(t);

    // デバッグ確認用の二値化画像をパブリッシュ
    sensor_msgs::msg::Image::SharedPtr img_thresholded_msg =
      cv_bridge::CvImage(color_msg->header, "mono8", img_thresholded).toImageMsg();
    image_thresholded_publisher_->publish(*img_thresholded_msg);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // ColorDetectorノードを登録してスピニングを実行
  rclcpp::spin(std::make_shared<ColorDetector>());
  rclcpp::shutdown();
  return 0;
}
