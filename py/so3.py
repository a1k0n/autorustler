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


def tensorhat(r):
    """ input shape: (3,n)
        output shape: (3, 3, n) """
    Z = np.zeros(r.shape[1])
    return np.array([
        [Z, -r[2], r[1]],
        [r[2], Z, -r[0]],
        [-r[1], r[0], Z]])


def tensorexp(r):
    """ returns a stack of rotation matrices as a tensor """
    """ r should be (3,n), n column vectors """
    theta = np.sqrt(np.sum(r*r, axis=0))  # shape = (n,)
    # note: the case where theta == 0 is not handled; we assume there is enough
    # noise and bias that this won't happen
    K = tensorhat(r / theta)  # shape = (3,3,n)
    KK = np.einsum('ijl,jkl->ikl', K, K)
    # Compute w/ Rodrigues' formula
    return np.eye(3)[:, :, np.newaxis] + np.sin(theta) * K + \
        (1 - np.cos(theta)) * KK


def diff(r, R, u):
    """ return Jacobian of exp(r_hat) * u
    R = exp(r) <- caller will already have this, so pass it in """
    # derivation: http://arxiv.org/abs/1312.0788
    # THIS IS WRONG! which even a rudimentary test will prove
    # for now use cv2.Rodrigues
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
