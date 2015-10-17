import autograd.numpy as np
import quat


def initial():
    q = np.array([0, 0, 0, 1], np.float32)  # unit quaternion orientation
    b_g = np.array([0, 0, 0], np.float32)   # gyroscope bias
    v_g = np.array([0, 0, 0], np.float32)   # velocity of IMU in global frame
    b_a = np.array([0, 0, 0], np.float32)   # accelerometer bias
    p_g = np.array([0, 0, 0], np.float32)   # position in global frame
    return (q, b_g, v_g, b_a, p_g)


def ode(state, measurement):
    q, bg, vg, ba, pg = state
    wm, am = measurement
    gravity = np.array([0, 0, -9.81])

    # we could subtract out Earth's coriolis force here, but there's no way
    # we're sensitive enough to notice it
    what = wm - bg
    ahat = am - ba

    dq = 0.5 * np.dot(quat.Omega(what), q)
    dvg = np.dot(quat.C(q).T, ahat) + gravity
    dpg = vg
    return (dq, dvg, dpg)


def propagate(state, measurement, dt):
    q, bg, vg, ba, pg = state
    dq, dvg, dpg = ode(state, measurement)
    # use Heun's method (trapezoidal rule)
    dq2, dvg2, dpg2 = ode(
        (q + dq*dt, bg, vg + dvg*dt, ba, pg + dpg*dt), measurement)
    q = q + 0.5*dt*(dq + dq2)
    vg = vg + 0.5*dt*(dvg + dvg2)
    pg = pg + 0.5*dt*(dpg + dpg2)
    q /= np.linalg.norm(q)
    return (q, bg, vg, ba, pg)
