from kinematics_helpers import *
from yefan_api.robot import *


def calculate(js):
    h = dot([Tz(d0), Rz(js[0]), Tz(d11), Tx(d12), Ry(js[1]), Tz(d2),
             Ry(-js[2]), Tz(d31), Rx(-js[3]), Tx(d4), Ry(-js[4]), Tx(d5), Rx(-js[5])])
    return h



