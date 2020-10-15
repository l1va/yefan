from sympy import *

def dot(matrices):
    res = one()
    for m in matrices:
        res = res * m
    return res


def one():
    return Matrix([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])


def Rx(a):
    return Matrix([[1, 0, 0, 0],
                   [0, cos(a), -sin(a), 0],
                   [0, sin(a), cos(a), 0],
                   [0, 0, 0, 1]])


def Ry(a):
    return Matrix([[cos(a), 0, sin(a), 0],
                   [0, 1, 0, 0],
                   [-sin(a), 0, cos(a), 0],
                   [0, 0, 0, 1]])


def Rz(a):
    return Matrix([[cos(a), -sin(a), 0, 0],
                   [sin(a), cos(a), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])


def Tx(d):
    return Matrix([[1, 0, 0, d],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])


def Ty(d):
    return Matrix([[1, 0, 0, 0],
                   [0, 1, 0, d],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])


def Tz(d):
    return Matrix([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, d],
                   [0, 0, 0, 1]])

