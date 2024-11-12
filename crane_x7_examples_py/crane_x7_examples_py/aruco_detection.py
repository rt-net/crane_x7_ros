# Copyright 2020 RT Corporation
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rclpy
from rclpy.node import Node

import numpy as np

# from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
import cv2
from cv2 import aruco
from cv_bridge import CvBridge
import tf2_ros

from crane_x7_examples_py.utils import rotation_matrix_to_quaternion


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('aruco_detection')
        self.image_subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.listener_callback, 10
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.listener_callback, 10
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.camera_info = None
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # 画像データをROSのメッセージからOpenCVの配列に変換
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding)
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)

        if self.camera_info:
            # ArUcoマーカのデータセットを読み込む
            # DICT_6x6_50は6x6ビットのマーカが50個収録されたもの
            MARKER_DICT = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

            # マーカーID
            ids = []

            # 画像座標系上のマーカ頂点位置
            corners = []

            # マーカの検出
            corners, ids, _ = aruco.detectMarkers(cv_img, MARKER_DICT)

            # マーカの検出数
            n_markers = len(ids)

            # カメラパラメータ
            CAMERA_MATRIX = np.array(self.camera_info['k']).reshape(3, 3)
            DIST_COEFFS = np.array(self.camera_info['d']).reshape(1, 5)

            # マーカ一辺の長さ 0.04 [m]
            MARKER_LENGTH = 0.04

            # マーカが一つ以上検出された場合、マーカの位置姿勢をtfで配信
            if n_markers > 0:
                for i in range(n_markers):
                    # 画像座標系上のマーカ位置を三次元のカメラ座標系に変換
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                                        corners, MARKER_LENGTH, CAMERA_MATRIX, DIST_COEFFS)
                    # tfの配信
                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = 'camera_color_optical_frame'
                    t.child_frame_id = 'target_' + str(ids[i][0])
                    t.transform.translation.x = tvecs[i][0][0]
                    t.transform.translation.y = tvecs[i][0][1]
                    t.transform.translation.z = tvecs[i][0][2]
                    # 回転ベクトル→回転行列
                    rotation_matrix, _ = cv2.Rodrigues(rvecs[i])
                    # 回転行列からクォータニオンを求める
                    q = rotation_matrix_to_quaternion(rotation_matrix)
                    t.transform.rotation.x = q.x
                    t.transform.rotation.y = q.y
                    t.transform.rotation.z = q.z
                    t.transform.rotation.w = q.w
                    self.tf_broadcaster.sendTransform(t)

    def camera_info_callback(self, msg):
        self.camera_info = msg


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
