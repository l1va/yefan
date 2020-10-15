import numpy as np
from math import sin, cos


def dot(matrices):
    res = one()
    for m in matrices:
        res = np.dot(res, m)
    return res


def one():
    return [[1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]]


def Rx(a):
    return [[1, 0, 0, 0],
            [0, cos(a), -sin(a), 0],
            [0, sin(a), cos(a), 0],
            [0, 0, 0, 1]]


def Ry(a):
    return [[cos(a), 0, sin(a), 0],
            [0, 1, 0, 0],
            [-sin(a), 0, cos(a), 0],
            [0, 0, 0, 1]]


def Rz(a):
    return [[cos(a), -sin(a), 0, 0],
            [sin(a), cos(a), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]]


def Tx(d):
    return [[1, 0, 0, d],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]]


def Ty(d):
    return [[1, 0, 0, 0],
            [0, 1, 0, d],
            [0, 0, 1, 0],
            [0, 0, 0, 1]]


def Tz(d):
    return [[1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, d],
            [0, 0, 0, 1]]
