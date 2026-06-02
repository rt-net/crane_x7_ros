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
from cv2 import aruco
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('aruco_detection')
        self.image_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        self.info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/color/camera_info')
        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.info_sub], 10)
        self.ts.registerCallback(self.camera_callback)

        # ArUcoマーカのデータセットを読み込む
        # DICT_6x6_50は6x6ビットのマーカが50個収録されたもの
        self.marker_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.bridge = CvBridge()

    def camera_callback(self, img_msg, info_msg):
        # 画像データをROSのメッセージからOpenCVの配列に変換
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding=img_msg.encoding)
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)

        # 画像座標系上のマーカ頂点位置
        corners = []
        # マーカID
        ids = []
        # マーカの検出
        corners, ids, _ = aruco.detectMarkers(cv_img, self.marker_dict)

        if ids is None:
            return
        # マーカの検出数
        n_markers = len(ids)

        # カメラパラメータ
        CAMERA_MATRIX = np.array(info_msg.k).reshape(3, 3)
        DIST_COEFFS = np.array(info_msg.d).reshape(1, 5)

        # マーカ一辺の長さ 0.04 [m]
        MARKER_LENGTH = 0.04

        # 画像座標系上のマーカ位置を三次元のカメラ座標系に変換
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, MARKER_LENGTH, CAMERA_MATRIX, DIST_COEFFS
        )

        # マーカの位置姿勢をtfで配信
        for i in range(n_markers):
            t = TransformStamped()
            t.header = img_msg.header
            t.child_frame_id = 'target_' + str(ids[i][0])
            t.transform.translation.x = tvecs[i][0][0]
            t.transform.translation.y = tvecs[i][0][1]
            t.transform.translation.z = tvecs[i][0][2]

            # 回転ベクトルをクォータニオンに変換
            marker_orientation_rot = Rotation.from_rotvec(rvecs[i][0])
            marker_orientation_quat = marker_orientation_rot.as_quat()
            t.transform.rotation.x = marker_orientation_quat[0]
            t.transform.rotation.y = marker_orientation_quat[1]
            t.transform.rotation.z = marker_orientation_quat[2]
            t.transform.rotation.w = marker_orientation_quat[3]

            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
