import math

import pytest

from vlm_nav_bridge.geometry import deproject, standoff_pose

FX = FY = 565.60
CX, CY = 320.0, 240.0


def test_deproject_principal_point_is_straight_ahead():
    assert deproject(320, 240, 2.0, FX, FY, CX, CY) == (0.0, 0.0, 2.0)


def test_deproject_right_is_positive_x_and_down_is_positive_y():
    X, Y, Z = deproject(600, 400, 3.0, FX, FY, CX, CY)
    assert X > 0 and Y > 0 and Z == 3.0
    assert X == pytest.approx((600 - 320) * 3.0 / FX)
    assert Y == pytest.approx((400 - 240) * 3.0 / FY)


def test_deproject_scales_linearly_with_depth():
    a = deproject(500, 300, 1.0, FX, FY, CX, CY)
    b = deproject(500, 300, 2.0, FX, FY, CX, CY)
    assert all(bi == pytest.approx(2 * ai) for ai, bi in zip(a, b))


@pytest.mark.parametrize("target,robot,expected", [
    ((5.0, 0.0), (0.0, 0.0), (4.4, 0.0, 0.0)),               # target ahead on +X
    ((0.0, 5.0), (0.0, 0.0), (0.0, 4.4, math.pi / 2)),       # target on +Y
    ((-5.0, 0.0), (0.0, 0.0), (-4.4, 0.0, math.pi)),         # target behind, robot turns round
    ((0.0, -5.0), (0.0, 0.0), (0.0, -4.4, -math.pi / 2)),    # target on -Y
])
def test_standoff_axis_cases(target, robot, expected):
    sx, sy, yaw = standoff_pose(target, robot, 0.6)
    assert sx == pytest.approx(expected[0])
    assert sy == pytest.approx(expected[1])
    assert abs(math.remainder(yaw - expected[2], 2 * math.pi)) < 1e-9


def test_standoff_is_radius_from_target_and_faces_it():
    target, robot = (3.0, 4.0), (-1.0, 1.5)
    sx, sy, yaw = standoff_pose(target, robot, 0.6)
    assert math.hypot(sx - target[0], sy - target[1]) == pytest.approx(0.6)
    # heading points from the standoff position toward the target
    assert math.cos(yaw) * (target[0] - sx) + math.sin(yaw) * (target[1] - sy) == pytest.approx(0.6)


def test_standoff_stays_on_the_robot_side():
    target, robot = (3.0, 4.0), (-1.0, 1.5)
    sx, sy, _ = standoff_pose(target, robot, 0.6)
    assert math.hypot(sx - robot[0], sy - robot[1]) < math.hypot(target[0] - robot[0], target[1] - robot[1])


def test_standoff_none_when_already_inside_radius_or_on_target():
    assert standoff_pose((1.0, 0.0), (0.0, 0.0), 0.6) is not None
    assert standoff_pose((0.5, 0.0), (0.0, 0.0), 0.6) is None
    assert standoff_pose((0.6, 0.0), (0.0, 0.0), 0.6) is None
    assert standoff_pose((2.0, 2.0), (2.0, 2.0), 0.6) is None
