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
// https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Cpp.html

#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"
#include "cv_bridge/cv_bridge.h"
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
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    auto cv_img = cv_bridge::toCvShare(msg, msg->encoding);
    cv::cvtColor(cv_img->image, cv_img->image, cv::COLOR_RGB2BGR);
    cv::Mat imageCopy;
    cv_img->image.copyTo(imageCopy);

    if (camera_info) {
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      // ArUcoマーカの辞書データを読み込む
      // DICT_6x6_50は6x6ビットのマーカが50個収録されたもの
      const auto MARKER_DICT = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
      cv::aruco::detectMarkers(imageCopy, MARKER_DICT, corners, ids);
      cv::Mat cameraMatrix, distCoeffs;
      cameraMatrix = cv::Mat(3, 3, CV_64F, camera_info->k.data());
      distCoeffs = cv::Mat(1, 5, CV_64F, camera_info->d.data());

      if (ids.size() > 0) {
        cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(
          corners, 0.04, cameraMatrix, distCoeffs, rvecs, tvecs);
        for (int i = 0; i < static_cast<int>(ids.size()); i++) {
          cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
        }

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "camera_color_optical_frame";
        t.child_frame_id = "target";
        t.transform.translation.x = tvecs[0][0];
        t.transform.translation.y = tvecs[0][1];
        t.transform.translation.z = tvecs[0][2];
        tf2::Quaternion q;
        cv::Mat cameraOri;
        cv::Rodrigues(rvecs[0], cameraOri);
        tf2::Matrix3x3 mattt = tf2::Matrix3x3(
          cameraOri.at<double>(0, 0),
          cameraOri.at<double>(0, 1),
          cameraOri.at<double>(0, 2),
          cameraOri.at<double>(1, 0),
          cameraOri.at<double>(1, 1),
          cameraOri.at<double>(1, 2),
          cameraOri.at<double>(2, 0),
          cameraOri.at<double>(2, 1),
          cameraOri.at<double>(2, 2));
        mattt.getRotation(q);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);
      }
    }
  }

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    camera_info = msg;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}
