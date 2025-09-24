import numpy as np


class PossitionController():
    def __init__(self, Kp = 3.0, Ki = 0.001, Kd = 3.5, max_integral=10.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.max_integral = max_integral

        self.prev_error = 0
        self.integral_error = 0

    def compute(self, current, target, dt):
        target = np.array(target)
        current = np.array(current)
        error = target - current
        
        P = self.Kp * error

        self.integral_error += (error * dt)
        self.integral_error = np.clip(self.integral_error, -self.max_integral, self.max_integral)
        I = self.Ki * self.integral_error

        D = self.Kd * ((error - self.prev_error)/dt)
        self.prev_error = error
        
        desired_velocity = P+I+D
        return desired_velocity # desired speed in the global coordinate system
