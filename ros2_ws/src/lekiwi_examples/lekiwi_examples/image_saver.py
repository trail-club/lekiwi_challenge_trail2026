"""手首カメラの画像を、サービスを叩いた瞬間に保存する例。

    # 端末 A: 常駐させる
    ros2 run lekiwi_examples image_saver

    # 端末 B: 保存したいタイミングで叩く
    ros2 service call /image_saver/save std_srvs/srv/Trigger

`robot.launch.py` が動いていることが前提です。

カラーと depth を購読し続けて、**最新の 1 枚だけ**を持ちます。サービスが
呼ばれたら、それを OpenCV の配列へ変換して保存します。まだ 1 枚も届いて
いなければ `success=False` と理由を返します。

保存先は `captured_images/example_rgb.png` と `example_depth.jpg`
（`docker/robot/compose.yaml` が `/captured_images` にマウントしています）。
depth は 16UC1 [mm] を 0-255 へ正規化したグレースケールにします。

★ **`cv_bridge` は使いません。** numpy 2 系では `imgmsg_to_cv2()` が
  SIGSEGV します（import は通るので気付きにくい）。詳細は
  `docs/development.md`。
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

OUTPUT_DIR = Path("/captured_images")
RGB_NAME = "example_rgb.png"
DEPTH_NAME = "example_depth.jpg"

COLOR_TOPIC = "/wrist_camera/wrist_camera/color/image_raw"
# align_depth の既定が false なので、カラーとは画角がずれる。揃えたいときは
# realsense を align_depth:=true で起動し、ここを
# /wrist_camera/wrist_camera/aligned_depth_to_color/image_raw にする。
DEPTH_TOPIC = "/wrist_camera/wrist_camera/depth/image_rect_raw"

# sensor_msgs/Image のエンコーディング -> (numpy 型, チャンネル数)
ENCODINGS = {
    "bgr8": (np.uint8, 3),
    "rgb8": (np.uint8, 3),
    "mono8": (np.uint8, 1),
    "mono16": (np.uint16, 1),
    "16UC1": (np.uint16, 1),
    "32FC1": (np.float32, 1),
}


def imgmsg_to_np(message: Image) -> np.ndarray:
    """sensor_msgs/Image を numpy 配列にする（OpenCV と同じ BGR 並び）。"""
    dtype, channels = ENCODINGS[message.encoding]
    array = np.frombuffer(message.data, dtype=dtype)
    if channels > 1:
        array = array.reshape(message.height, message.width, channels)
    else:
        array = array.reshape(message.height, message.width)
    # OpenCV は BGR を期待するので、rgb8 のときだけ入れ替える。
    return array[..., ::-1] if message.encoding == "rgb8" else array


class ImageSaver(Node):
    def __init__(self) -> None:
        super().__init__("image_saver")

        self._color: Image | None = None
        self._depth: Image | None = None

        # カメラは SENSOR_DATA（BEST_EFFORT）で購読する。
        self.create_subscription(
            Image, COLOR_TOPIC,
            lambda m: setattr(self, "_color", m), qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, DEPTH_TOPIC,
            lambda m: setattr(self, "_depth", m), qos_profile_sensor_data,
        )

        self.create_service(Trigger, "~/save", self._save_cb)
        self.get_logger().info(
            f"待機中。保存は  ros2 service call {self.get_name()}/save "
            f"std_srvs/srv/Trigger"
        )

    def _save_cb(self, _request, response):
        color, depth = self._color, self._depth
        if color is None or depth is None:
            missing = "color" if color is None else "depth"
            response.success = False
            response.message = f"{missing} の画像がまだ 1 枚も届いていない"
            self.get_logger().warning(response.message)
            return response

        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        images = (
            (OUTPUT_DIR / RGB_NAME, imgmsg_to_np(color)),
            # depth はそのままでは真っ黒なので、最小-最大で 0-255 へ引き伸ばす。
            (OUTPUT_DIR / DEPTH_NAME, cv2.normalize(
                imgmsg_to_np(depth), None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
            )),
        )
        saved = []
        for path, image in images:
            if not cv2.imwrite(str(path), image):
                response.success = False
                response.message = f"保存に失敗: {path}"
                self.get_logger().error(response.message)
                return response
            saved.append(f"{path} {image.shape}")
            self.get_logger().info(f"保存: {saved[-1]}")

        response.success = True
        response.message = " / ".join(saved)
        return response


def main() -> None:
    rclpy.init()
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
