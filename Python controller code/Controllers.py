import numpy as np


class Controller:
    action_min = 80
    action_max = 300
    bound = 3

    def clip(self, x):
        if -self.bound < x < self.bound:
            x = 0
        elif x > 0:
            x = np.clip(x, self.action_min, self.action_max)
        else:
            x = np.clip(x, -self.action_max, -self.action_min)
        return x


class PID(Controller):
    def __init__(self, _pid_coefs, bounds):
        self.pid_coefs = np.array(_pid_coefs)
        self.bounds = bounds

    def step(self, state, target_state):
        angle = state[0]
        # gate = 1008 * np.pi
        # if -gate <= angle <= gate:
        #     state *= np.array((0,1,0,1))

        speed_state = self.pid_coefs * (target_state - state)
        # print(speed_state)
        # speed_state = np.array([self.clip(speed) for speed in speed_state])
        speed = self.clip(speed_state.sum())
        if abs(state[0]) > self.bounds:
            speed = 0
        return speed

    def step_unsafe(self, state, target_state):
        speed_state = self.pid_coefs * (target_state - state)
        speed = self.clip(speed_state.sum())
        return speed

class Swing(Controller):
    def __init__(self, bounds, mode="manual"):
        self.action_min = 110
        self.action_max = 300

        self.bounds = bounds
        self.bound_coeff = 0.1
        self.bound_step = 0.05

        self.rightflag = True
        self.leftflag = False
        self.movingflag = False

        self.angle_last = -1
        self.angle_changed = True

        self.speed_last = -1
        self.speed_changed = True

        self.speed = 0
        self.mode = mode
        self.speed_t = 200

        if self.mode == "energy":
            self.energy = EnergyBased(1200)

    def step(self, state, speed_max):
        angle = state[0]
        angle_speed = state[2]
        position = state[1]



        if self.mode == "manual":
            speed_t = self.speed_t * self.bound_coeff
        elif self.mode == "energy":
            speed_t = np.clip(abs(self.energy.step(state)), 100, self.speed_t)
        else:
            raise ValueError("Unsupported mode")

        if self.movingflag:
            if position >= self.bounds * self.bound_coeff:
                self.bound_coeff = np.clip(self.bound_coeff + self.bound_step, 0, 1)
                self.rightflag = True
                self.movingflag = False
            elif position <= -self.bounds * self.bound_coeff:
                self.bound_coeff = np.clip(self.bound_coeff + self.bound_step, 0, 1)
                self.leftflag = True
                self.movingflag = False

        if abs(position) < self.bounds * self.bound_coeff:
            self.movingflag = True

        if self.rightflag:
            if self.speed_changed and (angle < 0):
                self.speed = -speed_t
                self.rightflag = False
                # self.movingflag = True
            else:
                if not (self.rightflag == self.leftflag):
                    self.speed = 0

        if self.leftflag:
            if self.speed_changed and (angle > 0):
                self.speed = speed_t
                self.leftflag = False
                # self.movingflag = True
            else:
                if not (self.rightflag == self.leftflag):
                    self.speed = 0

        if np.sign(self.speed_last) != np.sign(angle_speed):
            self.speed_changed = True
        else:
            self.speed_changed = False

        self.speed_last = angle_speed

        return self.clip(self.speed)

    def __getspeed_manual(self, state):
        return self.speed_t




class EnergyBased(Controller):
    def __init__(self, _coeff):
        self.coeff = _coeff
        self.m_p = 0.466/2
        self.m_c = 0.098
        self.l = 0.591/2
        self.g = 9.8

        self.E_target = 0.5 * self.m_p * self.g * self.l
        self.I_p = 4 / 3 * self.m_p * self.l**2

    def step(self, state):
        theta = state[0]
        omega = state[2]
        # vel = state[3]
        # sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)

        E_real = (self.I_p * omega ** 2 / 2) + self.E_target * (1 - cos_theta)
        action = self.coeff * (self.E_target - E_real) * np.sign(omega * cos_theta)
        # action = self.coeff * np.sign(omega * cos_theta)
        print(action)
        return self.clip(action)
