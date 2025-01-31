# Copyright 2025 RT Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from image_geometry import PinholeCameraModel
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('color_detection')
        self.image_subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10
        )
        self.depth_info_subscription = self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10
        )
        self.image_thresholded_publisher = self.create_publisher(
            Image, 'image_thresholded',  10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.camera_info = None
        self.depth_image = None

        self.bridge = CvBridge()

    def image_callback(self, msg):
        # カメラのパラメータを取得してから処理を行う
        if not self.camera_info or not self.depth_image:
            return

        # 青い物体を検出するようにHSVの範囲を設定
        # 周囲の明るさ等の動作環境に合わせて調整
        LOW_H = 100
        HIGH_H = 125
        LOW_S = 100
        HIGH_S = 255
        LOW_V = 30
        HIGH_V = 255

        # ウェブカメラの画像を受け取る
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding)

        # 画像をRGBからHSVに変換
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2HSV)

        # 画像の二値化
        img_thresholded = cv2.inRange(
            cv_img,
            (LOW_H, LOW_S, LOW_V),
            (HIGH_H, HIGH_S, HIGH_V)
        )

        # ノイズ除去の処理
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        img_thresholded = cv2.morphologyEx(
            img_thresholded, cv2.MORPH_OPEN, kernel
        )

        # 穴埋めの処理
        img_thresholded = cv2.morphologyEx(
            img_thresholded, cv2.MORPH_CLOSE, kernel
        )

        # 画像の検出領域におけるモーメントを計算
        moment = cv2.moments(img_thresholded)
        d_m01 = moment['m01']
        d_m10 = moment['m10']
        d_area = moment['m00']

        # 検出した領域のピクセル数が10000より大きい場合
        if d_area > 10000:
            # カメラモデル作成
            camera_model = PinholeCameraModel()

            # カメラのパラメータを設定
            camera_model.fromCameraInfo(self.camera_info)

            # 画像座標系における把持対象物の位置（2D）
            pixel_x = d_m10 / d_area
            pixel_y = d_m01 / d_area
            point = (pixel_x, pixel_y)

            # 補正後の画像座標系における把持対象物の位置を取得（2D）
            rect_point = camera_model.rectifyPoint(point)

            # カメラ座標系から見た把持対象物の方向（Ray）を取得する
            ray = camera_model.projectPixelTo3dRay(rect_point)

            # 把持対象物までの距離を取得
            # 把持対象物の表面より少し奥を掴むように設定
            DEPTH_OFFSET = 0.015
            cv_depth = CvBridge().imgmsg_to_cv2(
                self.depth_image, desired_encoding=self.depth_image.encoding)

            # カメラから把持対象物の表面までの距離
            front_distance = cv_depth[int(point[1]), int(point[0])] / 1000.0
            center_distance = front_distance + DEPTH_OFFSET

            # 距離を取得できないか遠すぎる場合は把持しない
            DEPTH_MAX = 0.5
            DEPTH_MIN = 0.2
            if center_distance < DEPTH_MIN or center_distance > DEPTH_MAX:
                self.get_logger().info(f'Failed to get depth at {point}.')
                return

            # 把持対象物の位置を計算
            object_position = [
                ray[0] * center_distance,
                ray[1] * center_distance,
                ray[2] * center_distance
            ]

            # 把持対象物の位置をTFに配信
            t = TransformStamped()
            t.header = msg.header
            t.child_frame_id = 'target_0'
            t.transform.translation.x = object_position[0]
            t.transform.translation.y = object_position[1]
            t.transform.translation.z = object_position[2]
            self.tf_broadcaster.sendTransform(t)

        # 閾値による二値化画像を配信
        img_thresholded_msg = self.bridge.cv2_to_imgmsg(
            img_thresholded, encoding='mono8'
        )
        self.image_thresholded_publisher.publish(img_thresholded_msg)

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, msg):
        self.depth_image = msg


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
