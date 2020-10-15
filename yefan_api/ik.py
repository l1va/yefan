# from sympy_helpers import *
from kinematics_helpers import *
from math import atan, atan2, sqrt, acos, pi
from robot import *
import fk
import numpy as np


def calculate(t):
    t2 = t
    t123 = dot([Tz(-d0), t2, Tx(-d5)])  # move wrist center to J5

    # find q123 first
    x = t123[0, 3]
    y = t123[1, 3]
    z = t123[2, 3]

    # t123s = dot([Rz(q1), Tz(d11), Tx(d12), Ry(q2), Tz(d2), Ry(q3 + dq3), Tx(d33)])
    # print(t123s)
    eps = 1e-9
    if abs(x) > eps:
        q1 = atan2(y, x)
    else:
        q1 = 0  # any
    # print(q1)

    d33 = sqrt(d31 * d31 + d4 * d4)

    # x = (d12 + d2*sin(q2)+d33*cos(q2+q3+dq))*cos(q1)
    # z = d11 + d2*cos(q2)-d33*sin(q2+q3+dq)

    x = x / cos(q1) - d12
    z = z - d11
    dq3 = atan2(d4, d31)

    # x^2 + z^2 = d2^2 + d3^2 + 2d2d3 cos(q3+dq)

    cc = (x ** 2 + z ** 2 - d2 ** 2 - d33 ** 2) / (2 * d2 * d33)
    q3 = acos(cc) - dq3
    # print(q3)

    q2 = pi / 2 - atan2(z, x) - atan2(d33 * sin(q3 + dq3), (d2 + d33 * cos(q3 + dq3)))
    # print(q2)
    q3 = -q3

    # And now find q456

    # q4 = Symbol('q4', real=True)
    # q5 = Symbol('q5', real=True)
    # q6 = Symbol('q6', real=True)
    # t456s = dot( [Rx(-q4), Ry(-q5), Rx(-q6)])
    # print(t456s)
    # [[cos(q5), sin(q5)*sin(q6), -sin(q5)*cos(q6), 0],
    # [sin(q4)*sin(q5), ..,.., 0],
    # [sin(q5)*cos(q4), ..,.., 0],
    #  [0, 0, 0, 1]]

    t123inv = np.linalg.inv(dot([Rz(q1), Tz(d11), Tx(d12), Ry(q2), Tz(d2), Ry(-q3), Tz(d31), Tx(d4)]))
    # print(t123inv)
    t456 = np.dot(t123inv, t123)
    # print(t456)
    nx = t456[0, 0]

    if abs(abs(nx) - 1) > eps:
        ny = t456[1, 0]
        nz = t456[2, 0]
        q4 = atan2(ny, nz)
        sx = t456[0, 1]
        ax = t456[0, 2]
        q6 = atan2(sx, -ax)
        if abs(sx) > eps:
            q5 = atan2(sx, nx * sin(q6))
            #print(q6, sx, nx, sin(q6), q5)
        else:
            q5 = atan2(ax, nx * cos(q6))
    else:
        q5 = acos(nx)
        q4 = 0  # any
        q6 = 0  # any

    # fff = fk.calculate([q1, q2, q3, q4, q5, q6])
    # if not np.allclose(fff, t):
    #     print(fff, t)

    return [q1, q2, q3, q4, q5, q6]


def main():
    calculate([[6.5995575e-01, -7.0813223e-02, 7.4795983e-01, 1.9912926e+03],
               [6.2244683e-01, 6.0904827e-01, -4.9154872e-01, 1.1441590e+03],
               [-4.2073549e-01, 7.8996562e-01, 4.4602237e-01, 1.7479431e+03],
               [0.0000000e+00, 0.0000000e+00, 0.0000000e+00, 1.0000000e+00]])


main()
