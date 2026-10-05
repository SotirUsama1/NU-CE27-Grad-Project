import math

import pytest

from semantic_world_manager.coordinate_utils import quaternion_to_yaw_deg, world_to_target


@pytest.mark.parametrize("yaw", [0.0, 90.0, 180.0, 270.0])
def test_quaternion_to_yaw_cardinal_angles(yaw):
    half_angle = math.radians(yaw) / 2.0
    assert quaternion_to_yaw_deg(0.0, 0.0, math.sin(half_angle), math.cos(half_angle)) == pytest.approx(yaw)


def test_world_to_map_applies_translation_and_rotation():
    loc, yaw = world_to_target(3.0, 7.0, 0.0, 90.0, (1.0, 2.0, 90.0))
    assert loc == pytest.approx([5.0, -2.0, 0.0])
    assert yaw == pytest.approx(0.0)
