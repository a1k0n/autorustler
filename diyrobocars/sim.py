import numpy as np
import cv2


steering_const = 8.0  # max steering angle
speed_const = 200  # max speed
T_w = 0.4
T_v = 2.

track = np.frombuffer(open("trackdata.f32").read(), np.float32)
track = track.reshape((4, -1))
# starting point is at 30, so rotate the track points so that it's in the front
# and also close the loop (repeat point 30 at the end)
track = np.hstack([track[:, 30:-1], track[:, 0:31]])
trackx = track[:2]
trackdx = track[2:4]
leftline = trackx + 20 * np.dot([[0, -1], [1, 0]], trackdx)
rightline = trackx + 20 * np.dot([[0, 1], [-1, 0]], trackdx)
trackx = np.vstack([trackx, np.zeros(track.shape[1]), np.ones(track.shape[1])])
leftline = np.vstack([leftline, np.zeros(track.shape[1]), np.ones(track.shape[1])])
rightline = np.vstack([rightline, np.zeros(track.shape[1]), np.ones(track.shape[1])])


def draw_track(img, P):
    # draw the track onto img using projection P
    # the track is represented as x, y, 0, 1 coordinates
    N = trackx.shape[1]
    trackproj = np.dot(P, trackx)
    trackproj[:2] /= trackproj[3]
    trackproj *= 16
    for i in range(0, N-1, 2):
        cv2.line(img,
                 (int(trackproj[0, i]), int(trackproj[1, i])),
                 (int(trackproj[0, i+1]), int(trackproj[1, i+1])),
                 (0, 0, 0), 3, shift=4)
        cv2.line(img,
                 (int(trackproj[0, i]), int(trackproj[1, i])),
                 (int(trackproj[0, i+1]), int(trackproj[1, i+1])),
                 (0, 255, 255), 1, shift=4)
    trackproj = np.dot(P, leftline)
    trackproj[:2] /= trackproj[3]
    trackproj *= 16
    for i in range(0, N-1):
        cv2.line(img,
                 (int(trackproj[0, i]), int(trackproj[1, i])),
                 (int(trackproj[0, i+1]), int(trackproj[1, i+1])),
                 (255, 255, 255), 1, shift=4)
    trackproj = np.dot(P, rightline)
    trackproj[:2] /= trackproj[3]
    trackproj *= 16
    for i in range(0, N-1):
        cv2.line(img,
                 (int(trackproj[0, i]), int(trackproj[1, i])),
                 (int(trackproj[0, i+1]), int(trackproj[1, i+1])),
                 (255, 255, 255), 1, shift=4)


def get_curvature(s):
    # FIXME: just make this a lookup table
    s %= track.shape[1] - 1
    si = np.int32(s)  # track point index (+1 hack to handle negatives)
    if si == track.shape[1] - 1:
        si = 0
    ts0 = track[:, si]
    ts1 = track[:, si+1]
    p0 = ts0[0] + 1j*ts0[1]
    p1 = ts1[0] + 1j*ts1[1]
    v0 = ts0[2] + 1j*ts0[3]
    v1 = ts1[2] + 1j*ts1[3]
    k = 1j * (v1 - v0) / (p0 - p1)
    return np.real(k)


def next_state(X, u, T=1.0/30):
    # unpack state and input
    while T > 0:
        ye, psi, vt, w, s = X
        X = np.copy(X)  # copy X for output
        K = get_curvature(s)
        throttle, steering = u

        # update s (note: points are spaced 11 units apart, hence 1/11 adjustment)
        dsdt = (1. / 11.) * vt * np.cos(psi) * (1.0 / (1 - ye*K))
        if dsdt == 0:
            dt = T
        elif dsdt > 0:
            # determine time left until we cross next segment
            nexts = np.floor(s + 1) - s
            dt = min(T, nexts / dsdt)
            X[4] += dsdt * dt
        else:
            # previous
            nexts = np.ceil(s - 1) - s
            dt = min(T, nexts / dsdt)
            X[4] += dsdt * dt
        assert dt > 0

        # update vt
        X[2] += dt * (speed_const * throttle - vt) / T_v
        vt = (vt + X[2]) * 0.5  # use midpoint

        # update w
        X[3] += dt * np.sign(vt) * (np.abs(vt) / speed_const)**0.6 * (steering_const * steering - w) / T_w
        w = (w + X[3]) * 0.5  # use midpoint

        # update psi, ye
        X[1] += dt * (w - vt * np.cos(psi) * (K / (1. - ye*K)))
        psi = (psi + X[1]) * 0.5
        X[1] %= 2*np.pi
        X[0] += dt * vt * np.sin(psi)
        ye = (X[0] + ye) * 0.5

        T -= dt

    return X


def draw_frame(X, u):
    # unpack state
    ye, psi, vt, w, s = X
    frame = np.zeros((480, 640, 3))

    s = np.mod(s, track.shape[1] - 1)
    si = np.int32(s)  # track point index (+1 hack to handle negatives)
    st = s - si  # fractional
    ts0 = track[:, si]
    ts1 = track[:, si+1]
    ts = st*ts1 + (1-st)*ts0  # interpolate along s
    ts[2:4] /= np.linalg.norm(ts[2:4], axis=0)  # renormalize the track normal
    dy = np.dot([[0, -1], [1, 0]], ts[2:4])
    xy = ts[:2] + ye*dy

    P = np.eye(4)  # project based on state
    P[:2, 3] -= xy

    R = np.eye(4)
    R[0, :2] = dy
    R[1, :2] = -ts[2:4]

    P = np.dot(R, P)
    P[:2] *= 3
    
    P[:2, 3] += np.float32([320, 240])

    draw_track(frame, P)

    # show virtual curvature on track
    K = get_curvature(s)
    r = 1.0 / K
    cv2.circle(frame,
               (int(16*(320 + 3*r - 3*ye)), 16*240),
               int(16*3*np.abs(r)),
               (255, 0, 0), 1, shift=4)

    # draw the car CG
    C = cv2.Rodrigues(np.float32([0, 0, psi]))[0][:2, :2]
    cv2.line(frame,
             (int(-4*16*C[0, 0] + 320*16), int(-4*16*C[1, 0] + 240*16)),
             (int(4*16*C[0, 0] + 320*16), int(4*16*C[1, 0] + 240*16)),
             (0, 255, 0), 1, shift=4)
    cv2.line(frame,
             (int(-16*16*C[0, 1] + 320*16), int(-16*16*C[1, 1] + 240*16)),
             (int(4*16*C[0, 1] + 320*16), int(4*16*C[1, 1] + 240*16)),
             (0, 0, 255), 1, shift=4)
    # cv2.circle(frame, (320, 240), 3, (255, 255, 255), 2)

    cv2.line(frame,
             (int(u[0] * 300 + 320), 460), (320, 460), (255, 255, 255), 5)
    cv2.line(frame,
             (int(u[1] * 300 + 320), 470), (320, 470), (255, 255, 0), 5)

    cv2.imshow('state', frame)


def control(X, nsteps, nrecur):
    # dumb N-step lookahead

    s0 = X[4]
    besta, bests, bestR = None, None, None
    for a in [-1.0, 0.0, 1.0]:
        for s in [-1.0, -0.5, 0.0, 0.5, 1.0]:
            nextx = X
            for _ in range(20):
                nextx = next_state(nextx, [a, s])
                if np.abs(nextx[0]) > 18:
                    # no going out of the track
                    nextx[4] = -1000
                    break
                # if np.abs(nextx[0] * nextx[5] - 1.0) < 0.5:
                #     # no crossing the radius of curvature
                #     nextx[4] = -1000
                #     break
            R = nextx[4] - s0
            if nrecur > 1:
                R += control(nextx, nsteps, nrecur - 1)[2]
            if bestR is None or R > bestR:
                besta, bests, bestR = a, s, R
    return besta, bests, bestR


def sim():
    X = np.zeros(5)
    X[0] = 10
    accel, steering = 1.0, 0.1
    while True:
        draw_frame(X, [accel, steering])
        k = cv2.waitKey(10)
        if k == ord('q'):
            break
        # X[4] += 0.1
        # print 'K', X[5]
        # steering = np.clip(X[2] * X[5] / steering_const - 0.0001*X[0], -1, 1)
        accel, steering, reward = control(X, 10, 1)
        print accel, steering, reward
        X = next_state(X, [accel, steering])
        print "s: %0.1f ye %0.2f psi %0.3f vt %0.1f" % (X[4], X[0], X[1], X[2])

if __name__ == '__main__':
    sim()

