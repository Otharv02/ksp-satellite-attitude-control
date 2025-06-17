"""
To apply control to reduce the angular velocity and align the camera.
"""
class PID:
    def __init__(self, kp, ki, kd):
        self.kp         = kp
        self.ki         = ki
        self.kd         = kd
        self.prev_error = 0
        self.integral   = 0

    def update(self, error, dt):
        self.integral           += error *dt                        # Fix long-term offset
        derivative               = (error - self.prev_error)/dt     # measure rate of chage of error
        self.prev_error          = error                            # current error 

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(min(output, 1), -1)
    


        