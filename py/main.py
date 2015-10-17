import parse
import so3
import matplotlib.pyplot as plt
import autograd.numpy as np

GRAD_THRESHOLD = 0.02


def GammaCorrect(im):
    return 255.0 * np.power(im.astype(np.float32) * (1.0 / 255.0), 2.2)


def GenPyramid(im, levels):
    pyr = [im]
    for l in range(levels-1):
        siz = tuple(d / 2 for d in im.shape)
        im2 = np.zeros(siz)
        h, w = im.shape
        im2 = 0.25 * (im[0::2, 0::2] +
                      im[1::2, 0::2] +
                      im[0::2, 1::2] +
                      im[1::2, 1::2])
        pyr.append(im2)
        im = im2
    return pyr


# TODO: combine with code below and return both value and gradients
def InterpolatedGradients(im, pts):
    """ return linearly interpolated intensity values in im
    for all points pts

    pts is assumed to be ([1xn], [1xn]) tuple of y and x indices
    """
    ptsi = tuple(p.astype(np.int32) for p in pts)  # truncated integer part
    ptsf = (pts[0] - ptsi[0], pts[1] - ptsi[1])  # fractional part

    # bilinear weights
    up = 1 - ptsf[0]
    down = ptsf[0]
    left = 1 - ptsf[1]
    right = ptsf[1]

    # image components
    ul = im[ptsi]
    ur = im[(ptsi[0], 1+ptsi[1])]
    dl = im[(1+ptsi[0], ptsi[1])]
    dr = im[(1+ptsi[0], 1+ptsi[1])]
    return np.vstack((
        left*(dl - ul) + right*(dr - ur),  # y gradient
        up*(ur - ul) + down*(dr - dl)))    # x gradient


def InterpolatedValues(im, pts):
    """ return linearly interpolated intensity values in im
    for all points pts

    pts is assumed to be ([1xn], [1xn]) tuple of y and x indices
    """
    ptsi = tuple(p.astype(np.int32) for p in pts)  # truncated integer part
    ptsf = (pts[0] - ptsi[0], pts[1] - ptsi[1])  # fractional part

    # bilinear weights
    up = 1 - ptsf[0]
    down = ptsf[0]
    left = 1 - ptsf[1]
    right = ptsf[1]

    # image components
    ul = im[ptsi]
    ur = im[(ptsi[0], 1+ptsi[1])]
    dl = im[(1+ptsi[0], ptsi[1])]
    dr = im[(1+ptsi[0], 1+ptsi[1])]
    return up*(left*ul + right*ur) + down*(left*dl + right*dr)


def ImageGradientMag(im):
    # TODO: smooth image first (gaussian or median filter)
    gy = im[1:, :-1] - im[:-1, :-1]
    gx = im[:-1, 1:] - im[:-1, :-1]
    return gx*gx + gy*gy


def MappablePoints(im):
    gy = im[1:, :-1] - im[:-1, :-1]
    gx = im[:-1, 1:] - im[:-1, :-1]
    mask = gx*gx + gy*gy > GRAD_THRESHOLD
    # smear down
    mask[1:, :] = mask[1:, :] + mask[:-1, :]
    # smear right
    mask[:, 1:] = mask[:, 1:] + mask[:, :-1]
    return mask


def w(px, py, d, r, T):
    R = so3.exp(r)
    X = np.dot(R, np.array([d * px, d * py, d])) + T
    return X[:2] / X[2]


# returns [dw/dR dw/dT dw/dd], 7x2
def dw(px, py, d, r, T):
    R = so3.exp(r)
    x0 = np.array([px, py, 1])
    X = np.dot(R, d * x0) + T
    # derivative of projection w = xy/z, as a matrix
    # dw/dk = dw/dX * dX/dk
    dwdX = np.array([
        [X[2], 0, -X[0]],
        [0, X[2], -X[1]]]) / (X[2]*X[2])
    dXdR = so3.diff(r, R, d * x0)
    dXdT = np.eye(3)
    dXdd = np.dot(R, x0).reshape((3, 1))
    return np.dot(dwdX, np.hstack((dXdR, dXdT, dXdd)))


def test_dw(x, y, z, r, T):
    h = 1e-7
    J = dw(x, y, z, r, T)
    print 'Jacobian dw'
    print J
    w0 = w(x, y, z, r, T)

    dwdrx = (w(x, y, z, r+np.array([h, 0, 0]), T) - w0) / h
    dwdry = (w(x, y, z, r+np.array([0, h, 0]), T) - w0) / h
    dwdrz = (w(x, y, z, r+np.array([0, 0, h]), T) - w0) / h

    dwdtx = (w(x, y, z, r, T+np.array([h, 0, 0])) - w0) / h
    dwdty = (w(x, y, z, r, T+np.array([0, h, 0])) - w0) / h
    dwdtz = (w(x, y, z, r, T+np.array([0, 0, h])) - w0) / h

    dwdd = (w(x, y, z+h, r, T) - w0) / h

    print 'numeric dw/drx', dwdrx, '%f' % np.linalg.norm(dwdrx - J[:, 0])
    print 'numeric dw/dry', dwdry, '%f' % np.linalg.norm(dwdry - J[:, 1])
    print 'numeric dw/drz', dwdrz, '%f' % np.linalg.norm(dwdrz - J[:, 2])
    print 'numeric dw/dtx', dwdtx, '%f' % np.linalg.norm(dwdtx - J[:, 3])
    print 'numeric dw/dty', dwdty, '%f' % np.linalg.norm(dwdty - J[:, 4])
    print 'numeric dw/dtz', dwdtz, '%f' % np.linalg.norm(dwdtz - J[:, 5])
    print 'numeric dw/dz', dwdd, '%f' % np.linalg.norm(dwdd - J[:, 6])


def PlanePrior(siz):
    D = np.ones(siz)
    D[:siz[0]/2, :] *= 20
    for i in range(siz[0]/2, siz[0]):
        y = (i - siz[0]/2 + 1)
        z = 20.0 / y
        D[i, :] = z
    return D


def PhotometricError(iref, inew, R, T, points, D):
    # points is a tuple ([y], [x]); convert to homogeneous
    siz = iref.shape
    npoints = len(points[0])
    f = siz[1]  # focal length, FIXME
    Xref = np.vstack(((points[1] - siz[1]*0.5) / f,  # x
                      (siz[0]*0.5 - points[0]) / f,  # y (left->right hand)
                      np.ones(npoints)))             # z = 1
    # this is confusingly written -- i am broadcasting the translation T to
    # every column, but numpy broadcasting only works if it's rows, hence all
    # the transposes
    # print D * Xref
    Xnew = (np.dot(so3.exp(R), (D * Xref)).T + T).T
    # print Xnew
    # right -> left hand projection
    proj = Xnew[0:2] / Xnew[2]
    p = (-proj[1]*f + siz[0]*0.5, proj[0]*f + siz[1]*0.5)
    margin = 10  # int(siz[0] / 5)
    inwindow_mask = ((p[0] >= margin) & (p[0] < siz[0]-margin-1) &
                     (p[1] >= margin) & (p[1] < siz[1]-margin-1))
    npts_inw = sum(inwindow_mask)
    if npts_inw < 10:
        return 1e6, np.zeros(6 + npoints)
    # todo: filter points which are now out of the window
    oldpointidxs = (points[0][inwindow_mask],
                    points[1][inwindow_mask])
    newpointidxs = (p[0][inwindow_mask], p[1][inwindow_mask])
    origpointidxs = np.nonzero(inwindow_mask)[0]
    E = InterpolatedValues(inew, newpointidxs) - iref[oldpointidxs]
    # dE/dk ->
    # d/dk r_p^2 = d/dk (Inew(w(r, T, D, p)) - Iref(p))^2
    # = -2r_p dInew/dp dp/dw dw/dX dX/dk
    # = -2r_p * g(w(r, T, D, p)) * dw(r, T, D, p)
    # intensity gradients for each point
    Ig = InterpolatedGradients(inew, newpointidxs)
    # TODO: use tensors for this
    # gradients for R, T, and D
    gradient = np.zeros(6 + npoints)
    for i in range(npts_inw):
        # print 'newidx (y,x) = ', newpointidxs[0][i], newpointidxs[1][i]
        # Jacobian of w
        oi = origpointidxs[i]
        Jw = dw(Xref[0][oi], Xref[1][oi], D[oi], R, T)
        # scale back up into pixel space, right->left hand coords to get
        # Jacobian of p
        Jp = f * np.vstack((-Jw[1], Jw[0]))
        # print origpointidxs[i], 'Xref', Xref[:, i], 'Ig', Ig[:, i], \
        #     'dwdRz', Jw[:, 2], 'dpdRz', Jp[:, 2]
        # full Jacobian = 2*E + Ig * Jp
        J = np.sign(E[i]) * np.dot(Ig[:, i], Jp)
        # print '2 E[i]', 2*E[i], 'Ig*Jp', np.dot(Ig[:, i], Jp)
        gradient[:6] += J[:6]
        # print J[:6]
        gradient[6+origpointidxs[i]] += J[6]

    print R, T, np.sum(np.abs(E)), npts_inw
    # return ((0.2*(npoints - npts_inw) + np.dot(E, E)), gradient)
    return np.sum(np.abs(E)) / (npts_inw), gradient / (npts_inw)
    # return np.dot(E, E), gradient


def test_PhotometricError(iref, inew, R, T, points, D):
    h = 1e-7
    hh = 1.0 / h
    E0, g0 = PhotometricError(iref, inew, R, T, points, D)
    Erx, Ery, Erz = 0, 0, 0
    Erx = hh * (PhotometricError(
        iref, inew, R + np.array([h, 0, 0]), T, points, D)[0] - E0)
    Ery = hh * (PhotometricError(
        iref, inew, R + np.array([0, h, 0]), T, points, D)[0] - E0)
    Erz = hh * (PhotometricError(
        iref, inew, R + np.array([0, 0, h]), T, points, D)[0] - E0)

    Etx = hh * (PhotometricError(
        iref, inew, R, T + np.array([h, 0, 0]), points, D)[0] - E0)
    Ety = hh * (PhotometricError(
        iref, inew, R, T + np.array([0, h, 0]), points, D)[0] - E0)
    Etz = hh * (PhotometricError(
        iref, inew, R, T + np.array([0, 0, h]), points, D)[0] - E0)

    n = np.nonzero(g0[6:])[0][0]
    print 'nonzero point', n
    D[n] += h
    Ed = hh * (PhotometricError(iref, inew, R, T, points, D)[0] - E0)

    print 'E', E0, 'g:', g0[:7]
    print 'numerical gR: (%f %f %f)' % (Erx, Ery, Erz)
    print 'numerical gT: (%f %f %f)' % (Etx, Ety, Etz)
    print 'numerical D: %f' % Ed
    print 'Error_R: (%f %f %f)' % (Erx - g0[0], Ery - g0[1], Erz - g0[2])
    print 'Error_T: (%f %f %f)' % (Etx - g0[3], Ety - g0[4], Etz - g0[5])
    print 'Error_D:', (Ed - g0[6 + n])


def LoadExampleFrames():
    """ Load example frames, where we can assume there was just Z movement, one
    unit forward"""
    frames = parse.ParseLog(open('../rustlerlog-BMPauR'))
    #for i in range(320):
    #    frames.next()
    f = (None,)
    imu = None
    while f[0] != 'img':
        imu = f
        f = frames.next()
    im1 = f[2]  # GammaCorrect(f[2])
    f = (None,)
    while f[0] != 'img':
        f = frames.next()
    im2 = f[2]  # GammaCorrect(f[2])
    return im1, im2, imu


def GetVideoTimeSeries():
    TS = []
    VIS_RY = []
    VIS_RX = []
    t0 = None
    lasthsum = None
    lastvsum = None
    hprior, vprior = 0, 0
    for msg in parse.ParseLog(open('../rustlerlog-BMPauR')):
        if msg[0] == 'img':
            _, ts, im = msg
            im = GammaCorrect(im)
            hsum = np.sum(im, axis=0)
            hsum -= np.mean(hsum)
            vsum = np.sum(im, axis=1)
            vsum -= np.mean(vsum)
            if t0 is None:
                t0 = ts
                lasthsum = hsum
                lastvsum = vsum
            hoffset = np.argmax(
                -2*np.arange(-80 - hprior, 81 - hprior)**2 +
                np.correlate(lasthsum, hsum[80:-80], mode='valid')) - 80
            voffset = np.argmax(
                -2*np.arange(-60 - vprior, 61 - vprior)**2 +
                np.correlate(lastvsum, vsum[60:-60], mode='valid')) - 60
            TS.append(ts - t0)
            VIS_RY.append(hoffset)
            VIS_RX.append(voffset)
            hprior, vprior = hoffset, voffset
            lasthsum = hsum
            lastvsum = vsum
    return TS, VIS_RY, VIS_RX


def GetTimeSeries():
    TS = []
    ERR = []
    GYRO = []
    ACCEL = []
    lastim = None
    t0 = None
    for msg in parse.ParseLog(open('../rustlerlog-BMPauR')):
        if msg[0] == 'img':
            _, ts, im = msg
            if t0 is None:
                t0 = ts
            im = GammaCorrect(im)
            if lastim is not None:
                TS.append(ts - t0)
                ERR.append(np.sum(np.abs(lastim-im)) / (320*240))
                GYRO.append(GYRO[-1])
                ACCEL.append(ACCEL[-1])
            lastim = im
        elif msg[0] == 'imu':
            _, ts, gyro, mag, accel = msg
            if t0 is None:
                t0 = ts
            TS.append(ts - t0)
            GYRO.append(gyro)
            ACCEL.append(accel)
            if len(ERR):
                ERR.append(ERR[-1])
            else:
                ERR.append(0)
    return np.array(TS), np.array(ERR), np.array(GYRO), np.array(ACCEL)


def Track():
    # procedure:
    #  - get all time series data
    #  - use stationary data (classified by image not moving) to find
    #    gyro/accel bias, etc
    #    - gyro temp compensation also
    #  - predict camera movement via IMU measurements
    #  - use feature point tracking visual odometry initialized by previous
    #    estimate depth of each feature point
    #  - iterate to refine camera pose / keypoint depth
    #  - feed back into kalman filter
    
    # later, introduce motion model from throttle/steering input measurements
    # into kalman filter, see if we can get less error
    pass


def main():
    im1, im2, _ = LoadExampleFrames()
    pyr1 = GenPyramid(im1, 5)
    pyr2 = GenPyramid(im2, 5)

    # plt.imshow(np.hstack((pyr1[4], pyr2[4])), cmap='gray')
    plt.imshow(pyr2[4] - pyr1[4])
    plt.show()


if __name__ == '__main__':
    main()
