class PDController:
    def __init__(self, Kp, Kd, dt):
        self.Kp = Kp
        self.Kd = Kd
        self.dt = dt
        self.previous_error = 0

    def update(self, error):
        derivative = (error - self.previous_error) / self.dt
        self.previous_error = error
        return self.Kp * error + self.Kd * derivative
        
   