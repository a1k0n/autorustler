import numpy as np
import cv2
import pickle


steering_const = 0.1  # max steering angle (curvature, 1/radius)
steering_maxrate = 20*0.2  # max steering rate (curvature * speed)
# steering_maxrate = 200*0.2
speed_const = 200  # max speed
T_w = 0.1  # 100ms lag in traction
T_v = 2.  # 2s full acceleration speed

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


def get_curvature(s, interp=False):
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
    k = np.real(1j * (v1 - v0) / (p0 - p1))
    if interp:
        ds = s - si
        return (1 - ds) * k + ds * get_curvature(s+1)
    return k


def next_state(X, u, T=1.0/30):
    # unpack state and input

    X = np.copy(X)  # copy X for output
    ye, psi, vt, w, s = X
    throttle, steering = u

    # update vt
    X[2] += T * (speed_const * throttle - vt) / T_v
    vt = (vt + X[2]) * 0.5  # use midpoint

    # update w
    # torque is proportional to f(v*steering_angle*steering_const) - w
    # where f is a funciton of limited friction
    # technically speed should decrease with friction * sin(steering angle) also
    # assume perfect lateral friction for now
    steer_rate = np.clip(vt * steering_const * steering,
                         -steering_maxrate, steering_maxrate)
    X[3] += T * (steer_rate - w) / T_w
    w = (w + X[3]) * 0.5  # use midpoint

    while T > 0:
        ye, psi, _, _, s = X
        K = get_curvature(s)

        # update s (note: points are spaced 11 units apart, hence 1/11 adjustment)
        dsdt = (1. / 11.) * vt * np.cos(psi) * (1.0 / (1 - ye*K))
        if dsdt == 0:
            dt = T
        elif dsdt > 0:
            # determine time left until we cross next segment
            nexts = np.floor(s + 1) - s
            dt = nexts / dsdt
            if dt < T:  # if it happens before our timestep, skip to next segment
                X[4] = np.floor(s + 1)
            else:
                dt = T
                X[4] += dsdt * dt
        else:
            # previous
            nexts = np.ceil(s - 1) - s
            dt = nexts / dsdt
            if dt < T:
                X[4] = np.ceil(s - 1)
            else:
                dt = T
                X[4] += dsdt * dt
        assert dt > 0

        # update psi, ye
        X[1] += dt * (w - vt * np.cos(psi) * (K / (1. - ye*K)))
        psi = (psi + X[1]) * 0.5
        X[1] %= 2*np.pi
        X[0] += dt * vt * np.sin(psi)
        # ye = (X[0] + ye) * 0.5

        T -= dt

    return X


def draw_frame(X, u, vidout=None):
    # unpack state
    ye, psi, vt, w, s = X
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

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

    if vidout is not None:
        vidout.write(frame)

    cv2.imshow('state', frame)


def base_control(X, nsteps):
    # dumb N-step lookahead

    s0 = X[4]
    besta, bests, bestR = None, None, None
    for a, s in [
            (-1.0, 0), (-1.0, -0.5), (-1.0, 0.5),
            (0.5, -1.0), (0.5, -0.75), (0.5, -0.5), (0.5, -0.25), (0.5, 0),
            (0.5, 0.25), (0.5, 0.5), (0.5, 0.75), (0.5, 1.0),
            (1.0, -1.0), (1.0, -0.75), (1.0, -0.5), (1.0, -0.25), (1.0, 0),
            (1.0, 0.25), (1.0, 0.5), (1.0, 0.75), (1.0, 1.0)]:
        nextx = X
        oob = False
        for _ in range(20):
            nextx = next_state(nextx, [a, s])
            # if np.abs(nextx[0] * nextx[5] - 1.0) < 0.5:
            #     # no crossing the radius of curvature
            #     nextx[4] = -1000
            #     break
            if np.abs(nextx[0]) > 18:
                oob = True
        R = nextx[4] - s0
        if oob:
            R -= 1000
        if bestR is None or R > bestR:
            besta, bests, bestR = a, s, R
    return besta, bests, bestR


data_S0 = []
data_A = []
data_R = []
data_S1 = []
SSadj = np.loadtxt("SSadj.txt")


def XtoS(X):
    ye, psi, v, w, s = X
    # S = np.float32([1, ye, np.cos(psi) - 1.0, np.sin(psi),
    #                 v - 120, w, get_curvature(s)])
    # whiten the input space
    # S[1:] = np.dot(S[1:], SSadj)
    k = get_curvature(s)
    ds = s - np.int(s)
    denom = 1 - k * ye
    S = np.float32([
        1, ye, ye*ye, v*np.sin(psi), v*np.cos(psi) / denom,
        w - v*k*np.cos(psi) / denom,

    ])
    return S


def Qlearn(W, X0, A, R, X1):
    # just save the state values up for now
    # we're going to have to build up a dataset and recompute the gram matrices
    # at each iteration since the Q learning targets (max a' Q(s, a')) change
    # each time we change Q!

    # ye, psi, vt, w, s = S
    S0 = XtoS(X0)
    S1 = XtoS(X1)
    data_S0.append(S0)
    data_A.append(A)
    data_R.append(R)
    data_S1.append(S1)
    return W


LSPI_W = np.loadtxt("lspi_w.txt")
def control_lspi(X):
    W2 = LSPI_W[28:30]
    W3 = LSPI_W[30:]
    S = XtoS(X)
    SS = np.outer(S, S)[np.tril_indices(7)]
    Ns = len(SS)

    def augment_action(SS, A):
        return np.hstack([SS, A * A, np.kron(A, SS)])

    def Q(SS, A):
        return np.dot(augment_action(SS, A), LSPI_W)

    A = np.zeros(2)
    if W2[0] < 0:
        ''' negative curvature; the max is where dQ/da = 0 '''
        A[0] = np.clip((-0.5 * np.dot(SS, W3[:Ns])) / W2[0], -1, 1)
    else:
        ''' positive curvature; either -1 or 1 is better '''
        a0 = [-1, 1]
        A[0] = a0[np.argmax([Q(SS, np.float32([a, 0])) for a in a0])]
            
    if W2[1] < 0:
        A[1] = np.clip((-0.5 * np.dot(SS, W3[Ns:])) / W2[1], -1, 1)
    else:
        a1 = [-1, 1]
        A[1] = a1[np.argmax([Q(SS, np.float32([0, a])) for a in a1])]

    return A


POLICY_W = np.loadtxt('policy_w.txt')
def control_cloned(X):
    S = XtoS(X)
    SS = np.outer(S, S)[np.tril_indices(7)]
    return np.clip(np.dot(SS, POLICY_W), -1, 1)


WV = np.zeros(28)   # value function approximation
WV_A = 0.01*np.eye(28)  # accumulated least squares matrices
WV_b = np.zeros(28)
WP1 = np.eye(2)   # policy A*A weights
#WP2 = 1e-5*np.random.randn(2, 28)  # policy A*S weights
WP2 = np.zeros((2, 28))

# WP1 = np.loadtxt("WP1.txt")
# WP2 = np.loadtxt("WP2.txt")

#WP2[0, 0] = 0.3  # positive acceleration bias
#WP2[1, 1] = -0.1  # turn against ye
#WP2[1, 19] = -0.1  # turn against vt sin(psi)
#WP2[1, 5] = -0.1  # counteract steering rate
grad_WP1 = np.zeros_like(WP1)
grad_WP1s = np.zeros_like(WP1)
grad_WP2 = np.zeros_like(WP2)
grad_WP2s = np.zeros_like(WP2)
grad_N = 0
noise_cov = np.eye(2)

def actor_critic_control(X):
    S = XtoS(X)
    SS = np.outer(S, S)[np.tril_indices(7)]
    # print 'V(s)=', np.dot(SS, WV)
    # fixme: make noise consistent, follow probability distribution
    noise = np.dot(noise_cov, np.random.randn(2))
    A = np.clip(np.dot(np.linalg.inv(WP1), np.dot(WP2, SS))
                + noise, -1, 1)
    return A


def actor_critic_update(X0, A, R, X1, gamma=0.99, learnrate=0.0001):
    global WP1, WP2, WV_A, WV_b, WV
    global grad_WP1, grad_WP2, grad_N
    global grad_WP1s, grad_WP2s
    global noise_cov
    S0 = XtoS(X0)
    S1 = XtoS(X1)
    SS0 = np.outer(S0, S0)[np.tril_indices(7)]
    SS1 = np.outer(S1, S1)[np.tril_indices(7)]

    deltaV = SS0 - gamma * SS1
    WV_A += np.outer(deltaV, deltaV)
    WV_b += R * deltaV
    WV = np.linalg.solve(WV_A, WV_b)

    advantage = R + gamma * np.dot(SS1, WV) - np.dot(SS0, WV)
    g1 = -0.5*np.outer(A, A) * advantage
    grad_WP1 += g1
    grad_WP1s += g1*g1
    g2 = np.outer(A, SS0) * advantage
    grad_WP2 += g2
    grad_WP2s += g2*g2
    grad_N += 1
    # if X0[4] > 4:
    #    print 'V(s0)', np.dot(SS0, WV), 'V(s1)', np.dot(SS1, WV), 'R', R
    #    print 'A', A, 'adv', advantage # , 'g_wp1\n', grad_WP1, 'g_wp2\n', grad_WP2
    noise_cov = np.linalg.inv(np.linalg.cholesky(
        WP1 + 1e-2 + np.eye(2))).T


def actor_critic_batch_update(learnrate=0.001):
    global WV_A, WV_b, WV, WP1, WP2
    global grad_N, grad_WP1, grad_WP2, grad_WP1s, grad_WP2s

    if grad_N > 10:
        WP1 += learnrate * grad_WP1
        WP2 += learnrate * grad_WP2
    grad_WP1 = np.zeros_like(WP1)
    grad_WP1s = np.zeros_like(WP1)
    grad_WP2 = np.zeros_like(WP2)
    grad_WP2s = np.zeros_like(WP2)
    grad_N = 0

    WV_A *= 0.999
    WV_b *= 0.999
    print WP1

    pass


def pid_control(X):
    ''' eq. 26 from page 10 of Alain Micaelli, Claude Samson. Trajectory
    tracking for unicycle-type and two-steering-wheels mobile robots. RR-2097,
    INRIA. 1993. '''

    # strategy: compute desired w / v ratio
    # maximize v s.t. limits on w
    # compute target w' and v'
    # overshoot the control slightly to compensate for underdamped response:
    # return k1*(v' - v), k2*(w' - w)

    kpy = 0.01
    kvy = 0.2
    ye, psi, v, w, s = X
    cpsi = np.cos(psi)
    spsi = np.sin(psi)
    k = get_curvature(s, True)
    dx = cpsi / (1.0 - k*ye)
    w_target = v * dx * (-ye * dx * kpy*cpsi + spsi*(k*spsi - kvy*cpsi) + k)

    v_target = speed_const

    if np.abs(w_target) <= steering_maxrate:
        # max acceleration
        print 'max accel', w_target, w, (w_target - w) / steering_const
        return 1.0, np.clip((w_target - w) / (v * steering_const + 1e-6), -1, 1)

    v_target = w_target / steering_maxrate
    print 'w_target', w_target, 'v_target', v_target
    return (np.clip((v_target - v) / speed_const, -1, 1),
            np.sign(w_target))

def sim():
    vidout = cv2.VideoWriter("sim.h264", cv2.VideoWriter_fourcc(
        'X', '2', '6', '4'), 30, (640, 480), True)
    maxR = 0
    xs = []
    us = []
    while True:
        X = np.zeros(5)
        X[0] = np.random.randn()*5
        X[1] = np.random.randn()*0.2
        accel, steering = 1.0, 0.1
        steps = 0
        while True:   # X[4] < track.shape[1]:
            #if X[4] > 40:
            draw_frame(X, [accel, steering], vidout)
            k = cv2.waitKey(10)
            if k == ord('q'):
                return
            # X[4] += 0.1
            # print 'K', X[5]
            # steering = np.clip(X[2] * X[5] / steering_const - 0.0001*X[0], -1, 1)
            # accel, steering = control_lspi(X)
            # accel, steering = control_cloned(X)
            # accel, steering, _ = base_control(X, 5)
            # print _
            # accel, steering = 0.5, 1
            xs.append(X)
            # accel, steering = actor_critic_control(X)
            accel, steering = pid_control(X)
            A = np.float32([accel, steering])
            us.append(A)
            newX = next_state(X, A)

            # R = newX[4] - X[4]
            # print accel, steering, R
            # actor_critic_update(X, A, R, newX)
            X = newX
            #if np.abs(newX[0]) > 20:
            #    # terminal state
            #    break
            steps += 1
        maxR = max(X[4], maxR)
        print 'episode reward', X[4], 'max', maxR
        actor_critic_batch_update()
        break

    # print "s: %0.1f ye %0.2f psi %0.3f vt %0.1f" % (X[4], X[0], X[1], X[2])

    open("LMPCdata.pkl", "w").write(
        pickle.dumps([xs, us]))

if __name__ == '__main__':
    sim()

