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
import message_filters
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detection')
        # 画像とカメラ情報のサブスクライバ
        self.color_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/aligned_depth_to_color/image_raw'
        )
        self.info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/color/camera_info')

        # タイムスタンプ同期を適用して、カラー画像、深度画像、パラメータ情報を結合して受信
        self.sync = message_filters.TimeSynchronizer(
            [self.color_sub, self.depth_sub, self.info_sub], 10
        )
        self.sync.registerCallback(self.sync_callback)

        # 二値化した確認画像のパブリッシャ
        self.image_thresholded_publisher = self.create_publisher(Image, 'image_thresholded', 10)

        # target_0のTF配信用ブロードキャスタ
        self.tf_broadcaster = TransformBroadcaster(self)
        self.bridge = CvBridge()

    def sync_callback(self, color_msg, depth_msg, info_msg):
        # 青い物体を検出するようにHSVの抽出範囲を設定
        # 周囲の明るさ等の環境や、検出させたい色によって微調整します
        LOW_H, HIGH_H = 100, 125
        LOW_S, HIGH_S = 100, 255
        LOW_V, HIGH_V = 30, 255

        # ROSトピック画像をOpenCVの画像配列に変換
        cv_img = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding=color_msg.encoding)

        # RGB画像からHSV画像空間に変換
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2HSV)

        # 青い色の範囲で二値化（白黒マスク画像を生成）
        img_thresholded = cv2.inRange(cv_img, (LOW_H, LOW_S, LOW_V), (HIGH_H, HIGH_S, HIGH_V))

        # モルフォロジー演算（オープニング処理）で孤立したノイズを除去
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        img_thresholded = cv2.morphologyEx(img_thresholded, cv2.MORPH_OPEN, kernel)

        # モルフォロジー演算（クロージング処理）で白抽出領域の中の隙間を穴埋め
        img_thresholded = cv2.morphologyEx(img_thresholded, cv2.MORPH_CLOSE, kernel)

        # 検出した白い領域全体のモーメント（重心）を計算
        moment = cv2.moments(img_thresholded)
        d_m01 = moment['m01']
        d_m10 = moment['m10']
        d_area = moment['m00']

        # 検出した領域があまりにも小さい場合（面積が10000ピクセル以下）は無視
        if d_area <= 10000:
            return

        # PinholeCameraModelオブジェクトを利用して3D位置方向を計算
        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(info_msg)

        # 2D画像面における対象物の重心中心座標を算出
        pixel_x = d_m10 / d_area
        pixel_y = d_m01 / d_area
        point = (pixel_x, pixel_y)

        # カメラレンズの歪みを補正した2D点座標を算出
        rect_point = camera_model.rectifyPoint(point)

        # カメラの中心からその2D点への3次元方向単位ベクトル（Ray）を算出
        ray = camera_model.projectPixelTo3dRay(rect_point)

        # 物体表面から把持したい深さへのオフセット
        DEPTH_OFFSET = 0.015
        cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding=depth_msg.encoding)

        # カメラから対象物表面までの直線的な奥行き（m単位）を算出
        front_distance = cv_depth[int(point[1]), int(point[0])] / 1000.0
        center_distance = front_distance + DEPTH_OFFSET

        # 測定範囲の制限チェック（有効な範囲内かつロボットの可動範囲50cm以内か）
        DEPTH_MAX = 0.5
        DEPTH_MIN = 0.2
        if center_distance < DEPTH_MIN or center_distance > DEPTH_MAX:
            self.get_logger().info(f'Failed to get depth at {point}.')
            return

        # 対象物の3次元位置姿勢の決定
        object_position = [
            ray[0] * center_distance,
            ray[1] * center_distance,
            ray[2] * center_distance,
        ]

        # 計算結果をtarget_0というTFフレーム名として配信
        t = TransformStamped()
        t.header = color_msg.header
        t.child_frame_id = 'target_0'
        t.transform.translation.x = object_position[0]
        t.transform.translation.y = object_position[1]
        t.transform.translation.z = object_position[2]
        self.tf_broadcaster.sendTransform(t)

        # デバッグ確認用の二値化画像をパブリッシュ
        img_thresholded_msg = self.bridge.cv2_to_imgmsg(img_thresholded, encoding='mono8')
        img_thresholded_msg.header = color_msg.header
        self.image_thresholded_publisher.publish(img_thresholded_msg)


def main(args=None):
    rclpy.init(args=args)

    # ColorDetectorノードを登録して実行
    node = ColorDetector()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
