import numpy as np
import cv2


steering_const = 0.7  # max steering angle
speed_const = 100  # max speed
T_w = 0.04
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


def next_state(X, u, dt=1.0/30):
    # unpack state and input
    ye, psi, vt, w, s, K = X
    throttle, steering = u

    X = np.copy(X)  # copy X for output
    # update vt
    X[2] += dt * (speed_const * throttle - vt) / T_v
    vt = (vt + X[2]) * 0.5  # use midpoint

    # update w
    X[3] += dt * (vt / speed_const)**0.8 * (steering_const * steering - w) / T_w
    w = (w + X[3]) * 0.5  # use midpoint

    # update psi, ye
    X[1] += dt * (w - vt * np.cos(psi) * (K / (1. - ye*K)))
    psi = (psi + X[1]) * 0.5
    X[1] %= 2*np.pi
    X[0] += dt * vt * np.sin(psi)
    ye = (X[0] + ye) * 0.5

    # update s
    ds = 1.0 / 11.0  # points are spaced 11 units apart
    X[4] += ds * dt * vt * np.cos(psi) * (1.0 / (1 - ye*K))

    return X


def get_curvature(s):
    s = np.mod(s, track.shape[1] - 1)
    si = np.int32(s)  # track point index (+1 hack to handle negatives)
    ts0 = track[:, si]
    ts1 = track[:, si+1]
    p0 = ts0[0] + 1j*ts0[1]
    p1 = ts1[0] + 1j*ts1[1]
    v0 = ts0[2] + 1j*ts0[3]
    v1 = ts1[2] + 1j*ts1[3]
    k = 1j * (v1 - v0) / (p0 - p1)
    return np.real(k)


def draw_frame(X):
    # unpack state
    ye, psi, vt, w, s, K = X
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

    cv2.imshow('state', frame)


def sim():
    X = np.zeros(6)
    X[0] = 10
    accel, steering = 0.2, 0.1
    while True:
        X[5] = get_curvature(X[4])
        draw_frame(X)
        k = cv2.waitKey(10)
        if k == ord('q'):
            break
        # X[4] += 0.1
        # X[5] = get_curvature(X[4])
        # print 'K', X[5]
        steering = -0.1*X[0] - X[3]
        X = next_state(X, [accel, steering])
        print "s: %0.1f ye %0.2f psi %0.3f vt %0.1f" % (X[4], X[0], X[1], X[2])

if __name__ == '__main__':
    sim()

