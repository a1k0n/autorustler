import autograd.numpy as np


def hat(r):
    return np.array([
        [0, -r[2], r[1]],
        [r[2], 0, -r[0]],
        [-r[1], r[0], 0]])


def exp(r):
    """ matrix exponential under the special orthogonal group SO(3) ->
    converts Rodrigues 3-vector r into 3x3 rotation matrix R"""
    theta = np.linalg.norm(r)
    if (theta == 0):
        return np.eye(3)
    K = hat(r / theta)
    # Compute w/ Rodrigues' formula
    return np.eye(3) + np.sin(theta) * K + \
        (1 - np.cos(theta)) * np.dot(K, K)


def diff(r, R, u):
    """ return Jacobian of exp(r_hat) * u
    R = exp(r) <- caller will already have this, so pass it in """
    # derivation: http://arxiv.org/abs/1312.0788
    uhat = hat(u)
    rnorm = np.dot(r, r)
    if rnorm == 0:
        return -uhat
    return -np.dot(np.dot(R, uhat), (
        np.outer(r, r) + np.dot(R.T - np.eye(3), hat(r)))) / rnorm


def test_diff(r, u):
    h = 1e-7
    R = exp(r)
    J = diff(r, R, u)
    print 'Jacobian'
    print J
    r0 = np.dot(R, u)
    drdx = (np.dot(exp(r + np.array([h, 0, 0])), u) - r0) / h
    drdy = (np.dot(exp(r + np.array([0, h, 0])), u) - r0) / h
    drdz = (np.dot(exp(r + np.array([0, 0, h])), u) - r0) / h

    print 'drdx', drdx, '%f' % np.linalg.norm(drdx - J[:, 0])
    print 'drdy', drdy, '%f' % np.linalg.norm(drdy - J[:, 1])
    print 'drdz', drdz, '%f' % np.linalg.norm(drdz - J[:, 2])
