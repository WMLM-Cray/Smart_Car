import math
from common.params import CarParams

def to_cmd_vel(v, steer, p=CarParams()):
    return float(v), float(v * math.tan(steer) / p.wheelbase)
