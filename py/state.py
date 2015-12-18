import autograd.numpy as np
import so3


def initial():
    r_g = np.array([0, 0, 0], np.float32)   # global to imu frame rotation
    b_g = np.array([0, 0, 0], np.float32)   # gyroscope bias
    v_g = np.array([0, 0, 0], np.float32)   # velocity of IMU in global frame
    b_a = np.array([0, 0, 0], np.float32)   # accelerometer bias
    p_g = np.array([0, 0, 0], np.float32)   # position in global frame
    a_s = 1.0  # accelerometer scale
    return (r_g, b_g, v_g, b_a, p_g, a_s)


def ode(state, measurement):
    rg, bg, vg, ba, pg, a_s = state
    wm, am = measurement
    gravity = np.array([0, 0, -9.81])

    # we could subtract out Earth's coriolis force here, but there's no way
    # we're sensitive enough to notice it
    what = wm - bg
    ahat = (am - ba) * a_s

    dvg = np.dot(so3.exp(-rg), ahat) - gravity
    dpg = vg
    return (what, dvg, dpg)


def propagate(state, measurement, dt):
    rg, bg, vg, ba, pg, a_s = state
    dr, dvg, dpg = ode(state, measurement)
    # use Heun's method (trapezoidal rule)
    dr2, dvg2, dpg2 = ode(
        (rg + dr*dt, bg, vg + dvg*dt, ba, pg + dpg*dt, a_s), measurement)
    rg = rg + 0.5*dt*(dr + dr2)
    vg = vg + 0.5*dt*(dvg + dvg2)
    pg = pg + 0.5*dt*(dpg + dpg2)
    return (rg, bg, vg, ba, pg)
