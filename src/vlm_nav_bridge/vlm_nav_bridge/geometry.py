"""Pure geometry for pixel_to_goal. No ROS imports, so it can be unit-tested anywhere."""

import math


def deproject(u, v, d, fx, fy, cx, cy):
    """Pixel + depth -> 3D point in the camera optical frame (pinhole model).

    Args:
        u, v:   pixel column and row
        d:      depth along the optical axis, in metres
        fx, fy: focal lengths in pixels
        cx, cy: principal point in pixels

    Returns:
        (X, Y, Z) in metres. Optical frame: +Z out of the lens, +X right, +Y down.
    """
    X = (u - cx) * d / fx
    Y = (v - cy) * d / fy
    Z = d
    return X, Y, Z


def standoff_pose(target_xy, robot_xy, radius):
    """Pick a position `radius` metres short of the target, on the robot's side, facing it.

    Args:
        target_xy: (x, y) of the target in the map frame
        robot_xy:  (x, y) of the robot in the map frame
        radius:    standoff distance in metres

    Returns:
        (x, y, yaw) in the map frame, yaw in radians from +X, or None when the
        robot is already within `radius` of the target (or exactly on it), so the
        caller does not send a goal that would make it reverse.
    """
    tx, ty = target_xy
    rx, ry = robot_xy
    dx, dy = rx - tx, ry - ty
    dist = math.hypot(dx, dy)
    if dist <= radius or dist == 0.0:
        return None
    ux, uy = dx / dist, dy / dist          # unit vector from target toward robot
    sx, sy = tx + radius * ux, ty + radius * uy
    yaw = math.atan2(ty - sy, tx - sx)     # face the target
    return sx, sy, yaw
