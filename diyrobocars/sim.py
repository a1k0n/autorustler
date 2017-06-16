import numpy as np
import cv2
import pickle


steering_const = 1/1.  # max steering angle (curvature, 1/radius)
tire_maxa = 9

speed_const = 10  # max speed
T_w = 0.2  # 100ms lag in traction
# T_v = 2.  # 2s full acceleration speed
T_v = 1.  # 2s full acceleration speed

lanewidth = 1.8

trackx = np.loadtxt("track_x.txt", comments=",").reshape((-1, 2)).T
tracku = np.loadtxt("track_u.txt", comments=",").reshape((-1, 2)).T
trackk = np.loadtxt("track_k.txt", comments=",")

trackN = trackx.shape[1]
trackx = np.hstack([trackx, trackx[:, :1]])  # wrap
tracku = np.hstack([tracku, tracku[:, :1]])
trackk = np.concatenate([trackk, trackk[:1]])
track_spacing = np.mean(np.linalg.norm(trackx[:, 1:] - trackx[:, :-1], axis=0))
print 'track spacing', track_spacing
leftline = trackx - lanewidth/2 * tracku
rightline = trackx + lanewidth/2 * tracku

trackx = np.vstack([trackx, np.zeros(trackN+1), np.ones(trackN+1)])
leftline = np.vstack([leftline, np.zeros(trackN+1), np.ones(trackN+1)])
rightline = np.vstack([rightline, np.zeros(trackN+1), np.ones(trackN+1)])

# load pre-computed racing line
raceline_ye = np.loadtxt("raceline_ye.txt", comments=",")
raceline_psie = np.loadtxt("raceline_psie.txt", comments=",")
raceline_k = np.loadtxt("raceline_k.txt", comments=",")
raceline_ds = np.loadtxt("raceline_ds.txt", comments=",")


def draw_track(img, P):
    # draw the track onto img using projection P
    # the track is represented as x, y, 0, 1 coordinates
    N = trackN + 1  # FIXME
    trackproj = np.dot(P, trackx)
    trackproj[:2] /= trackproj[3]
    trackproj *= 16
    # draw dotted centerline
    for i in range(0, N-1, 2):
        cv2.line(img,
                 (int(trackproj[0, i]), int(trackproj[1, i])),
                 (int(trackproj[0, i+1]), int(trackproj[1, i+1])),
                 (128, 128, 128), 3, shift=4)
        cv2.line(img,
                 (int(trackproj[0, i]), int(trackproj[1, i])),
                 (int(trackproj[0, i+1]), int(trackproj[1, i+1])),
                 (0, 255, 255), 1, shift=4)

    # draw left line
    trackprojL = np.dot(P, leftline)
    trackprojL[:2] /= trackprojL[3]
    trackprojL *= 16
    for i in range(0, N-1):
        cv2.line(img,
                 (int(trackprojL[0, i]), int(trackprojL[1, i])),
                 (int(trackprojL[0, i+1]), int(trackprojL[1, i+1])),
                 (255, 255, 255), 1, shift=4)
    # right line
    trackprojR = np.dot(P, rightline)
    trackprojR[:2] /= trackprojR[3]
    trackprojR *= 16
    for i in range(0, N-1):
        cv2.line(img,
                 (int(trackprojR[0, i]), int(trackprojR[1, i])),
                 (int(trackprojR[0, i+1]), int(trackprojR[1, i+1])),
                 (255, 255, 255), 1, shift=4)
    # start line
    cv2.line(img,
             (int(trackprojL[0, 0]), int(trackprojL[1, 0])),
             (int(trackprojR[0, 0]), int(trackprojR[1, 0])),
             (255, 255, 255), 2, shift=4)


def get_curvature(s, interp=False):
    s %= trackN
    si = np.int32(s)  # track point index (+1 hack to handle negatives)
    k = trackk[si]
    if interp:
        ds = s - si
        return (1 - ds) * k + ds * trackk[si+1]
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
    # where f is a function of limited friction
    # technically speed should decrease with friction * sin(steering angle)
    # also assume perfect lateral friction for now
    if vt == 0:
        steer_rate = 0
    else:
        # w = kv = a/v
        # a = clip(kv^2, -amax, amax)
        # w = min(amax/v, kmax*v)    amax/v = kmax*v
        # amax = kmax v^2
        # v_wmax = sqrt(amax/kmax) --> sqrt(m^2/s^2), ok

        # wmax = amax/v_wmax = kmax*v_wmax
        #      = amax*sqrt(kmax/amax) = kmax*sqrt(amax/kmax)
        #      = sqrt(amax*kmax) = sqrt(amax*kmax) = sqrt(m/s^2 * 1/m) = sqrt(1/s^2) = 1/s ok

        # could also use tanh for a differentiable, smooth transition
        lat_accel = np.clip(steering_const * steering * vt * vt,
                            -tire_maxa, tire_maxa)
        steer_rate = lat_accel / vt
    X[3] += T * (steer_rate - w) / T_w
    w = (w + X[3]) * 0.5  # use midpoint

    while T > 0:
        ye, psi, _, _, s = X
        K = get_curvature(s)

        # update s (note: points are spaced 11 units apart, hence 1/11 adjustment)
        dsdt = (1.0 / track_spacing) * vt * np.cos(psi) * (1.0 / (1 - ye*K))
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


def draw_frame(X, T, u, vidout=None):
    # unpack state
    ye, psi, vt, w, s = X
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    s = np.mod(s, trackN)
    si = np.int32(s)  # track point index (+1 hack to handle negatives)
    st = s - si  # fractional
    ts0 = trackx[:, si]
    ts1 = trackx[:, si+1]
    tu0 = tracku[:, si]
    tu1 = tracku[:, si+1]
    ts = st*ts1 + (1-st)*ts0  # interpolate along s
    tu = st*tu1 + (1-st)*tu0  # interpolate along s
    tu /= np.linalg.norm(tu, axis=0)  # renormalize the track normal
    xy = ts[:2] + ye*tu

    P = np.eye(4)  # project based on state
    P[:2, 3] -= xy

    R = np.eye(4)
    R[0, :2] = tu
    R[1, :2] = np.dot([[0, -1], [1, 0]], tu)

    P = np.dot(R, P)
    P[:2] *= 30
    
    P[:2, 3] += np.float32([320, 240])

    draw_track(frame, P)

    # show virtual curvature on track
    K = get_curvature(s, True)
    if False:  # something is messed up here
        r = 1.0 / K
        cv2.circle(frame,
                   (int(16*(320 + 3*r - 3*ye)), 16*240),
                   int(16*3*np.abs(r)),
                   (255, 255, 0), 1, shift=4)

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

    cv2.putText(frame, "s: %0.1f" % (s * track_spacing), (0, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(frame, "v: %0.1f" % vt, (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(frame, "T: %0.2f" % T, (0, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

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
    # ds = s - np.int(s)
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


def pid_control(X, T=1.0/30):
    ''' eq. 26 from page 10 of Alain Micaelli, Claude Samson. Trajectory
    tracking for unicycle-type and two-steering-wheels mobile robots. RR-2097,
    INRIA. 1993. '''

    # strategy: compute desired w / v ratio
    # maximize v s.t. limits on w
    # compute target w' and v'
    # overshoot the control slightly to compensate for underdamped response:
    # return k1*(v' - v), k2*(w' - w)

    kpy = 0.4
    kvy = 1.0
    ye, psi, v, w, s = X

    si = np.int32(s) % trackN
    # ds = s - si
    lane_offset = raceline_ye[si]
    psi_offset = -raceline_psie[si]

    cpsi = np.cos(psi - psi_offset)
    spsi = np.sin(psi - psi_offset)
    # k = get_curvature(s, True)
    k = raceline_k[si]
    dx = cpsi / (1.0 - k*ye)

    k_target = dx * (-(ye - lane_offset) * dx * kpy*cpsi + spsi*(k*spsi - kvy*cpsi) + k)

    # w = kv = a/v
    # a = clip(kv^2, -amax, amax)
    # we want to max out |a|
    # |k|v^2 = amax
    # v = sqrt(amax/|k|)

    s_lookahead = (si + np.arange(1, 20)) % trackN
    k_lookahead = np.abs(raceline_k[s_lookahead])
    maxv_lookahead = np.minimum(speed_const, np.sqrt(tire_maxa / k_lookahead))
    # backtrack from lookahead to make sure we brake in time
    vmax = speed_const
    for i in range(18, -1, -1):
        # apply full braking backward in time
        # v' = v + dt * (-speed_const - v) / T_v
        # v' = v(1 - dt/Tv) - speed_const*dt/Tv
        # (v' + speed_const*dt/Tv) / (1 - dt/Tv) = v
        # TODO: compute distance between specific landmarks when following the
        # racing line
        # for now, just use the track spacing
        # v = ds / dt; dt = ds / v
        # dt = track_spacing / vmax  # give extra time for a safety margin
        dt = raceline_ds[s_lookahead][i] / vmax
        vmax = min(speed_const, (vmax + speed_const * dt/T_v) / (1 - dt/T_v))
        vmax = min(vmax, maxv_lookahead[i])


    # compute a velocity limit at each point, and then
    # work backwards at maximum braking to propagate velocity limit to current
    # time

    v_target = np.minimum(vmax, np.sqrt(tire_maxa / np.abs(k_target)))

    # v' = v + dt * (speed_const * throttle - v) / T_v
    # throttle = (T_v * v' / dt + v) / speed_const
    throttle = np.clip((T_v * (v_target - v) / T + v) / speed_const, -1, 1)

    print 'k_target', k_target, 'v_target', v_target, 'vmax', vmax, 'a', v_target**2 * k_target
    return throttle, np.clip(k_target / steering_const, -1, 1)


def sim():
    vidout = cv2.VideoWriter("sim.h264", cv2.VideoWriter_fourcc(
        'X', '2', '6', '4'), 30, (640, 480), True)
    xs = []
    us = []
    Rs = []
    done = False
    episodes = []
    T = 0
    dt = 1.0 / 30
    while not done:
        X = np.zeros(5)
        X[0] = np.clip(np.random.randn()*10, -0.8, 0.8)  # initial ye
        X[1] = np.random.randn()*0.2  # initial psi
        #X[2] = np.random.rand() * speed_const  # initial v
        #if np.random.rand() > 0.0:  # 30% of the time, start at beginning
        #    # X[4] = np.random.rand() * trackN  # initial s
        #    X[4] = np.random.rand() * 5 + 70  # initial s
        s0 = X[4]
        accel, steering = 1.0, 0.1
        steps = 0
        while True:
            #if X[4] > 40:
            draw_frame(X, T, [accel, steering], vidout)
            #k = cv2.waitKey(10)
            k = cv2.waitKey()
            if k == ord('q'):
                done = True
                break
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
            if np.abs(newX[0]) > 20:
                # terminal state
                Rs.append([-1])  # negative terminal state
                break
            #if X[4] >= 80:  # trackN - 1:  # positive terminal state
            #    Rs.append([1])
            #    break
            Rs.append([0])
            steps += 1
            T += dt
        if not done:
            episodes.append(np.hstack([xs, us, Rs]))
            xs = []
            us = []
            Rs = []

        ds = trackN - s0
        print 'episode reward:', steps, 'steps /', ds, float(steps)/ds
        actor_critic_batch_update()

    # print "s: %0.1f ye %0.2f psi %0.3f vt %0.1f" % (X[4], X[0], X[1], X[2])

    open("episodes.pkl", "w").write(pickle.dumps(episodes))

if __name__ == '__main__':
    sim()

