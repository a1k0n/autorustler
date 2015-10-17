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


def magcal_residual(MAG, a, mb):
    """ residual from all observations given magnetometer eccentricity, bias,
    gyro bias, and gyro scale"""

    A = np.array([
        [a[0], a[1], a[2]],
        [0,    a[3], a[4]],
        [0,    0,    a[5]]
    ])

    mag = np.dot(MAG - mb, A)
    return np.mean(np.abs(1 - np.einsum('ji,ji->j', mag, mag)))


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
