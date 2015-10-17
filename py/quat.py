import autograd.numpy as np


def cross(w):
    return np.array([
        [0, -w[2], w[1]],
        [w[2], 0, -w[0]],
        [-w[1], w[0], 0]
    ])


def Omega(w):
    return np.array([
        [0, w[2], -w[1], w[0]],
        [-w[2], 0, w[0], w[1]],
        [w[1], -w[0], 0, w[2]],
        [-w[0], -w[1], -w[2], 0]
    ])


def C(q):
    i, j, k, r = q
    return np.array([
        [1 - 2*j*j - 2*k*k, 2*(i*j - k*r),     2*(i*k + j*r)],
        [2*(i*j + k*r),     1 - 2*i*i - 2*k*k, 2*(j*k - i*r)],
        [2*(i*k - j*r), 2*(j*k + i*r),         1 - 2*i*i - 2*j*j]
    ])
