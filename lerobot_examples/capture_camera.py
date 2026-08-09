"""
SO101付属カメラから画像を取得するサンプル。
カメラのインデックスは lerobot_examples/config.toml の [camera] セクションで設定。

実行方法:
    uv run python capture_camera.py

操作:
    s キー: 画像をファイルに保存
    q キー: 終了
"""

from pathlib import Path

import cv2
import numpy as np
import tomllib

from lerobot.cameras.opencv import OpenCVCamera
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig

CONFIG_PATH = Path(__file__).parent / "config.toml"


def main():
    with open(CONFIG_PATH, "rb") as f:
        cfg = tomllib.load(f)

    cam_cfg = cfg["camera"]
    config = OpenCVCameraConfig(
        index_or_path=cam_cfg["index"],
        fps=cam_cfg["fps"],
        width=cam_cfg["width"],
        height=cam_cfg["height"],
    )
    camera = OpenCVCamera(config)
    camera.connect()
    print("カメラ接続完了。's' で保存、'q' で終了。")

    save_dir = Path("captured_images")
    save_dir.mkdir(exist_ok=True)
    save_count = 0

    try:
        while True:
            frame: np.ndarray = camera.read()

            # lerobotはRGBで返すのでOpenCVの表示用にBGRへ変換
            bgr_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            cv2.imshow("SO101 Camera", bgr_frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("s"):
                filename = save_dir / f"frame_{save_count:04d}.png"
                cv2.imwrite(str(filename), bgr_frame)
                print(f"保存しました: {filename}")
                save_count += 1

    finally:
        camera.disconnect()
        cv2.destroyAllWindows()
        print("終了しました。")


if __name__ == "__main__":
    main()
