# The "Electronic Speed Control" (ESC) in this car isn't really a speed
# control, more of a motor current control. At 50% input, it turns the motor on
# 50% of the time and freewheels the other 50% of the time. At -50% input
# (braking), it shorts the motor 50% of the time and freewheels 50% of the
# time.

# We can characterize its response with the following differential equation:
# dv/dt = k1*DC - k2*DC*v - k3*v
# where DC is the duty cycle and v is velocity.

# k1 is the acceleration constant from the motor (torque, technically, but
# we're working in linear space -- k1 is battery voltage * motor constant /
# inertia all rolled into one)

# k2 is the back-emf constant; as the motor speeds up it acts like a generator
# and counteracts applied voltage, but only while our PWM signal is high so
# it is also modulated by DC

# k3 is friction slowing everything down proportional to speed.

import numpy as np
from numpy import exp


def initial_state():
    # TODO: get initial s over smbus
    x = np.array([0, 0, np.log(1000), 0, 0])
    P = np.diag([0.1, 0.1, 0.1, 0.1, 0.1])**2
    return x, P


def predict(x, P, direction, duty_cycle, dt=1.0/30):
    ''' return predicted state x and covariance P given direction (1 or 0) and
    duty cycle (0..1) '''

    s, v, l_1, l_2, l_3 = x
    k1, k2, k3 = exp(l_1), exp(l_2), exp(l_3)
    u_tau, u_v = duty_cycle, direction

    xk = np.array([
        dt*(dt*(u_tau*u_v*k1 - u_tau*v*k2 - v*k3)/2 + v) + s,
        dt*(u_tau*u_v*k1 - u_tau*v*k2 - v*k3) + v,
        l_1,
        l_2,
        l_3
    ])

    F = np.array([
        [1, dt*(dt*(-k2*u_tau - k3)/2 + 1), dt**2*k1*u_tau*u_v/2, -dt**2*k2*u_tau*v/2, -dt**2*k3*v/2],
        [0, dt*(-k2*u_tau - k3) + 1, dt*k1*u_tau*u_v, -dt*k2*u_tau*v, -dt*k3*v],
        [0, 0, 1, 0, 0], [0, 0, 0, 1, 0], [0, 0, 0, 0, 1]])
        
    Pk = np.dot(F, np.dot(P, F.T)) + np.diag([0.1*dt, 0.1*dt, 1e-2, 1e-2, 1e-2])**2

    return xk, Pk


def update(x, P, new_s):
    s, v, l_1, l_2, l_3 = x

    y = new_s - s
    R = 1  # measurement noise, +/- 1 tick?
    S = P[0, 0] + R
    K = P[:, 0] / S
    # print 'y', y, 'S', S, 'K', K, 'update', 'y', K * y
    x += K * y
    I = np.eye(5)
    I[:, 0] -= K
    P = np.dot(I, P)
    return x, P


def control(x, v_target, dt=1.0/30):
    ''' returns direction (0 or 1) and duty cycle (0..1) to reach v_target from
    state x '''
    s, v, l_1, l_2, l_3 = x
    k1, k2, k3 = exp(l_1), exp(l_2), exp(l_3)

    # 0 = v_thresh + (k3*dt - 1)*v
    v_thresh = (1 - k3*dt)*v
    d = 1 if v_target > v_thresh else 0

    return d, np.clip((v_target + (k3*dt - 1)*v) / ((k1*d - k2*v)*dt), 0, 1)


if __name__ == '__main__':
    np.set_printoptions(suppress=True)

    x, P = initial_state()
    k1 = 4.
    k2 = 3.
    k3 = 2.
    s = 0
    v = 0
    dt = 1.0/30

    def step(V, dc):
        global s, v, x, P
        dv = dc*V*k1 - dc*k2*v - k3*v
        s += (v + dv*dt/2)*dt
        v += dv*dt
        x, P = predict(x, P, V, dc)
        x, P = update(x, P, s)
        # print 'in', V, dc, 'true s, v', s, v
        # print 'estimated state', x[0], x[1], np.exp(x[2:])

    v_max = k1/(k2 + k3)
    for i in range(1000):
        target_v = v_max * (1 + np.sin(i*0.01))/2
        d, dc = control(x, target_v)
        step(d, dc)
        print d, dc, target_v, v, x[1], np.exp(x[2:])
    print 'final covariance P'
    print P
