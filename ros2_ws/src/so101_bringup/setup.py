import os
from glob import glob

from setuptools import find_packages, setup

package_name = "so101_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml") + glob("config/*.xacro")),
        (os.path.join("share", package_name, "control"), glob("control/*.xacro")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="maintainer",
    maintainer_email="maintainer@example.com",
    description="LeRobot-backed ROS 2 bringup for the standalone SO-101 follower arm.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            # ★ ここはハードウェアに触るものだけ。逆運動学・リーチ・
            #   キーボード操作は lekiwi_examples へ移した。
            "so101_lerobot_bridge = so101_bringup.lerobot_bridge:main",
        ],
    },
)
