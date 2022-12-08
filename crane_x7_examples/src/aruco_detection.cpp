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
// https://github.com/ros-planning/moveit2_tutorials/blob
// /a547cf49ff7d1fe16a93dfe020c6027bcb035b51/doc/move_group_interface
// /src/move_group_interface_tutorial.cpp

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"
#include "cv_bridge/cv_bridge.h"
using std::placeholders::_1;

class ImageSubscriber : public rclcpp::Node
{
  public:
    ImageSubscriber()
    : Node("aruco_detection")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", 10, std::bind(&ImageSubscriber::topic_callback, this, _1));
    }

  private:
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
      auto cv_img = cv_bridge::toCvShare(msg, msg->encoding);
      cv::cvtColor(cv_img->image, cv_img->image, cv::COLOR_RGB2BGR);

      cv::Mat image, imageCopy;
      cv_img->image.copyTo(imageCopy);
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f> > corners;
      cv::aruco::detectMarkers(imageCopy, dictionary, corners, ids);
      // if at least one marker detected
      if (ids.size() > 0)
          cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
      cv::imshow("out", imageCopy);
      cv::waitKey(1);

    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}