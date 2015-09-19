import parse
import so3
import matplotlib.pyplot as plt
import numpy as np
# import theano.tensor as T

GRAD_THRESHOLD = 0.01


def GammaCorrect(im):
    return np.power(im * (1.0 / 255.0), 2.2)


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
    mask[1:, :] |= mask[:-1, :]
    # smear right
    mask[:, 1:] |= mask[:, :-1]
    return np.nonzero(mask)


def w(px, py, d, r, T):
    R = so3.exp(r)
    X = np.dot(R, np.array([d * px, d * py, d])) + T
    # print X, X[:2] / X[2]
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


def PhotometricError(iref, inew, R, T, points, D):
    # points is a tuple ([y], [x]); convert to homogeneous
    siz = iref.shape
    npoints = len(points[0])
    f = siz[1]  # focal length, FIXME
    Xref = np.vstack(((points[1] - siz[1]*0.5) / f,
                      (siz[0]*0.5 - points[0]) / f,
                      np.ones(npoints))) * D
    # this is confusingly written -- i am broadcasting the translation T to
    # every column, but numpy broadcasting only works if it's rows, hence all
    # the transposes
    print Xref
    Xnew = (np.dot(Xref.T, so3.exp(R)) + T).T
    print Xnew
    proj = Xnew[0:2] / Xnew[2]
    p = (-proj[1]*f + siz[0]*0.5, proj[0]*f + siz[1]*0.5)
    print points, p
    inwindow_mask = ((p[0] >= 0) & (p[0] < siz[0]-1) &
                     (p[1] >= 0) & (p[1] < siz[1]-1))
    npts = sum(inwindow_mask)
    print 'inwindow:', npts
    # todo: filter points which are now out of the window
    E = (InterpolatedValues(inew, (p[0][inwindow_mask], p[1][inwindow_mask])) -
         iref[(points[0][inwindow_mask], points[1][inwindow_mask])])
    return np.dot(E, E) / (npts - 1)


def LoadExampleFrames():
    frames = parse.ParseLog(open('../rustlerlog-BMPauR'))
    for i in range(90):
        frames.next()
    im1 = GammaCorrect(frames.next())
    im2 = GammaCorrect(frames.next())
    return im1, im2


def main():
    im1, im2 = LoadExampleFrames()
    pyr1 = GenPyramid(im1, 5)
    pyr2 = GenPyramid(im2, 5)

    # plt.imshow(np.hstack((pyr1[4], pyr2[4])), cmap='gray')
    plt.imshow(pyr2[4] - pyr1[4])
    plt.show()


if __name__ == '__main__':
    main()
