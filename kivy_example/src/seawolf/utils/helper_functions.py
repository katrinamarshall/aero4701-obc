import math


def quat_to_pitch_roll(w, x, y, z):
    # Converts a quaternion to pitch and roll
    # roll (y-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = -math.atan2(sinr_cosp, cosr_cosp) * (180 / math.pi)

    # pitch (x-axis rotation)
    sinp = 2 * (w * y - z * x)
    if math.fabs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp) * (
            180 / math.pi
        )  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp) * (180 / math.pi)
    return pitch, roll
