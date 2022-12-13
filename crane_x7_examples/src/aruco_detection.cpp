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
      image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", 10, std::bind(&ImageSubscriber::image_callback, this, _1));

      camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/color/camera_info", 10, std::bind(&ImageSubscriber::camera_info_callback, this, _1));
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
    cv::Ptr<cv::aruco::Dictionary> dictionary
    = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      auto cv_img = cv_bridge::toCvShare(msg, msg->encoding);
      cv::cvtColor(cv_img->image, cv_img->image, cv::COLOR_RGB2BGR);
      cv::Mat imageCopy;
      cv_img->image.copyTo(imageCopy);

      if(camera_info)
      {
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;
        cv::aruco::detectMarkers(imageCopy, dictionary, corners, ids);
        cv::Mat cameraMatrix, distCoeffs;
        cameraMatrix = cv::Mat(3, 3, CV_64F, camera_info->k.data());
        distCoeffs = cv::Mat(1, 5, CV_64F, camera_info->d.data());

        if(ids.size() > 0)
        {
          cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
          std::vector<cv::Vec3d> rvecs, tvecs;
          cv::aruco::estimatePoseSingleMarkers(
            corners, 0.04, cameraMatrix, distCoeffs, rvecs, tvecs);
          for(int i=0; i<int(ids.size()); i++)
          {
            cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
          }
        }
      }
      cv::imshow("out", imageCopy);
      cv::waitKey(1);
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