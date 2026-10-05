import math


def quaternion_to_yaw_deg(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return round(math.degrees(math.atan2(siny_cosp, cosy_cosp)) % 360.0, 2)


def world_to_target(x, y, z, yaw_deg, transform):
    offset_x, offset_y, offset_yaw_deg = transform
    theta = math.radians(offset_yaw_deg)
    dx = x - offset_x
    dy = y - offset_y
    target_x = dx * math.cos(theta) + dy * math.sin(theta)
    target_y = -dx * math.sin(theta) + dy * math.cos(theta)
    target_yaw = (yaw_deg - offset_yaw_deg) % 360.0
    return [target_x, target_y, z], target_yaw
