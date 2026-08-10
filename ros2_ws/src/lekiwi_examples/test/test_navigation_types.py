import math

import pytest

from lekiwi_examples.navigation_types import PlanarPose, approach_pose_from_landmark


def test_approach_pose_keeps_standoff_and_faces_landmark():
    landmark = PlanarPose(2.0, 3.0, math.pi / 2.0)

    approach = approach_pose_from_landmark(landmark, 0.6)

    assert approach.x == pytest.approx(2.0)
    assert approach.y == pytest.approx(2.4)
    assert approach.yaw == pytest.approx(math.pi / 2.0)


def test_explicit_final_yaw_is_independent_from_landmark_yaw():
    landmark = PlanarPose(2.0, 3.0, math.pi / 2.0)

    approach = approach_pose_from_landmark(landmark, 0.5, final_yaw=0.0)

    assert approach.x == pytest.approx(1.5)
    assert approach.y == pytest.approx(3.0)
    assert approach.yaw == pytest.approx(0.0)


@pytest.mark.parametrize("standoff", [-0.1, math.inf, math.nan])
def test_invalid_standoff_is_rejected(standoff):
    with pytest.raises(ValueError):
        approach_pose_from_landmark(PlanarPose(0.0, 0.0, 0.0), standoff)
