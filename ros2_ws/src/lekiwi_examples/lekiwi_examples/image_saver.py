"""手首カメラの画像を、サービスを叩いた瞬間に保存するノード。

    # 端末 A: 常駐させる
    ros2 run lekiwi_examples image_saver

    # 端末 B: 保存したいタイミングで叩く
    ros2 service call /image_saver/save std_srvs/srv/Trigger

★ `robot.launch.py` が動いていることが前提。

保存先は `captured_images/example_rgb.png` と `example_depth.jpg`
（`docker/robot/compose.yaml` が `/captured_images` にマウントしている）。

────────────────────────────────────────────────────────────────────────
★ なぜサービスにするか
────────────────────────────────────────────────────────────────────────
このノードは**購読しっぱなしで最新の 1 枚だけ持つ**。変換と保存は
サービスが呼ばれた瞬間にやる。こうすると:

* 撮りたい瞬間に**カメラと同期を取らなくてよい**。すでに手元にある
* 保存の判断が**呼ぶ側**にある。別のプログラム（例: example_sequence）から
  好きなところで叩ける
* **1 枚も届いていなければ `success=False`** で理由を返せる。
  黙って古い画像や壊れた画像を書かない

★ **`cv_bridge` は使わない。** numpy 2 系では `imgmsg_to_cv2()` が SIGSEGV する
  （import は通るので気付きにくい）。理由と代替は `docs/development.md`。
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

#: 保存先。docker/robot/compose.yaml がホストの captured_images/ を張っている。
OUTPUT_DIR = Path("/captured_images")
RGB_NAME = "example_rgb.png"
DEPTH_NAME = "example_depth.jpg"

COLOR_TOPIC = "/wrist_camera/wrist_camera/color/image_raw"
# ★ align_depth の既定は false なので、RGB とは画角がずれる。
#   揃えたいなら realsense を align_depth:=true で起動し、ここを
#   /wrist_camera/wrist_camera/aligned_depth_to_color/image_raw にする。
DEPTH_TOPIC = "/wrist_camera/wrist_camera/depth/image_rect_raw"

# ★ ROS パラメータにしていないのは、これが**最小構成の例**だから。
#   実際に運用するノード（reach_to_point / base_driver / teleop_keyboard）は
#   YAML + declare_parameter を使う。理由は docs/development.md。
#   なお --symlink-install なので、ここを書き換えれば再ビルド無しで効く。

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
    """sensor_msgs/Image -> numpy（OpenCV と同じ BGR 並び）。

    ★ `cv_bridge` の代わり。やっていることはエンコーディングを見て
      `bytes` を numpy へ整形するだけなので、依存なしで書ける。
    """
    dtype, channels = ENCODINGS[message.encoding]
    array = np.frombuffer(message.data, dtype=dtype)
    if channels > 1:
        array = array.reshape(message.height, message.width, channels)
    else:
        array = array.reshape(message.height, message.width)
    # cv2 は BGR を期待する。rgb8 のときだけ入れ替える。
    return array[..., ::-1] if message.encoding == "rgb8" else array


class ImageSaver(Node):
    def __init__(self) -> None:
        super().__init__("image_saver")

        # ★ 最新の 1 枚だけ持つ。溜めない。
        self._color: Image | None = None
        self._depth: Image | None = None
        # ★ SENSOR_DATA (BEST_EFFORT)。publisher が RELIABLE でも繋がる。
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
        # ★ depth は 16UC1 [mm]（0 は無効値）。そのままでは真っ黒なので
        #   最小-最大で 0-255 へ引き伸ばしてグレースケールにする。
        images = (
            (OUTPUT_DIR / RGB_NAME, imgmsg_to_np(color)),
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
        # ★ 購読もサービスも spin の中で進む。スレッドは要らない。
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
