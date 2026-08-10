import os
from glob import glob

from setuptools import find_packages, setup

package_name = "lekiwi_examples"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="maintainer",
    maintainer_email="maintainer@example.com",
    description="Application-layer examples that run on top of robot.launch.py.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            # 最小構成の例。アームを動かして前へ進む
            "example_sequence = lekiwi_examples.example_sequence:main",
            # Nav2 NavigateToPose を競技用のリトライ付きAPIで呼ぶ最小例
            "navigation_demo = lekiwi_examples.navigation_demo:main",
            # 手首カメラの画像をサービス呼び出しで保存する
            "image_saver = lekiwi_examples.image_saver:main",
            # map 上の点へアームを伸ばす（RViz の "Publish Point"）
            "reach_to_point = lekiwi_examples.reach_to_point:main",
            # ベースとアームを同時にキーボード操作する
            "teleop_keyboard = lekiwi_examples.teleop_keyboard:main",
            # デカルト座標でアーム手先をジョグする（下 2 つで 1 組）
            "cartesian_jog = lekiwi_examples.cartesian_jog:main",
            "cartesian_keyboard = lekiwi_examples.keyboard_input:main",
        ],
    },
)
