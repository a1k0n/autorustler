import autograd.numpy as np
import so3
import parse


def GetIMU():
    TS, GYRO, TEMP, MAG, ACCEL = [], [], [], [], []
    for entry in parse.ParseLog(open("../rustlerlog-BMPauR")):
        if entry[0] == 'img':
            continue
        elif entry[0] != 'imu':
            continue
        _, ts, gyro, mag, accel = entry
        TS.append(ts)
        # gyro (rads/sec)
        GYRO.append(np.array(gyro[:3], np.float32) * np.pi / (180 * 14.375))
        TEMP.append(gyro[3])
        # magnetometer (in some random uT/LSB count units)
        MAG.append(mag.astype(np.float32))
        # accel (m/s^2)
        ACCEL.append(accel.astype(np.float32) * 9.81 / 256.0)
    return (np.array(TS), np.array(GYRO), np.array(TEMP),
            np.array(MAG), np.array(ACCEL))


TS, GYRO, TEMP, MAG, ACCEL = GetIMU()


def Amatrix(a):
    return np.array([
        [a[0], a[1], a[2]],
        [0,    a[3], a[4]],
        [0,    0,    a[5]]
    ])


def Ainv(x):
    a, b, c, d, e, f = x[:6]
    a = 1.0/a
    d = 1.0/d
    f = 1.0/f
    return np.array([
        [a, -a*b*d, -a*b*e*d*f - a*c*f],
        [0, d, -d*e*f],
        [0, 0, f]
    ])


def magcal_residual(X, a, mb):
    """ residual from all observations given magnetometer eccentricity, bias,
    gyro bias, and gyro scale"""

    # (x-c)T A^T A (x-c) = 1
    # x^T Ax - 2x^T Ac + c^T Ac = 1

    # a b c | x' = ax + by + cz
    # 0 d e | y' = dy + ez
    # 0 0 f | z' = fz
    # z = 1/f z'
    # y = 1/d (y' - e/f z')
    # x = 1/a (x' - b/d(y' - e/f z') - c/f z')
    #   = 1/a (x' - b/d y' - (be/df - c/f) z')
    # (x-c) A^T A (x-c)
    # [(A x) - (A c)]^2 - 1 = 0

    # y = A(x-c)
    # y /= ||y||
    # q(x; A, c) = (A^-1 (y+c) - x)^2

    Y = np.dot(X - mb, Ainv(a)).T
    Y /= np.linalg.norm(Y, axis=0)
    # Y /= np.sqrt(np.sum(np.square(Y), axis=0))
    Y = np.dot(Y.T, Amatrix(a)) + mb
    return np.mean(np.sum(np.square(X - Y), axis=1))


def magcal_residual_old(X, a, mb):
    mag = np.dot(X - mb, Ainv(a))
    return np.mean(np.square(1 - np.sum(mag*mag, axis=1)))


def magcal_residual2(a, mb, gb, gs):
    """ residual from all observations given magnetometer eccentricity, bias,
    gyro bias, and gyro scale"""

    A = np.array([
        [a[0], a[1], a[2]],
        [0,    a[3], a[4]],
        [0,    0,    a[5]]
    ])

    mag = np.dot(MAG - mb, A)
    dt = TS[1:] - TS[:-1]
    w = gs * (GYRO[1:] - gb)
    C = so3.tensorexp(w.T * dt)
    rot_mag = np.einsum('ijl,lj->li', C, mag[:-1])
    return np.mean(np.abs(1 - np.einsum('ji,ji->j', mag[1:], rot_mag)))

# (x-u)^T A (x-u) - 1
# x^T A (x-u) - u^T A (x-u) - 1
# x^T A x - x^T A u - u^T A x + u^T A u - 1
# x^T A x - (2 u^T A) x + (u^T A u - 1) = 0

# i wonder if a more numerically stable parameterization would be to use 1/diag
# ...nope


def initial_state(Xa, Xg):
    """ estimate initial orientation and biases given a series of measurements
    during which the IMU was stationary """

    g = 9.81
    bg = np.mean(Xg, axis=0)

    a = np.mean(Xa, axis=0)
    na = np.linalg.norm(a)
    sa = g / na
    a /= na

    rot = np.cross(np.array([0, 0, -1.0]), a)

    # print 'accel scale', sa
    # print 'gyro bias', bg
    # print 'initial orientation', rot
    # print np.dot(so3.exp(rot), np.array([0, 0, -1.0])), a
    return sa, bg, rot
