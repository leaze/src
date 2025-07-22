class PIDController:
    def __init__(self, kp, ki, kd, integral_limit=0.0, output_limit=100.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = None

    def compute(self, error, current_time):
        # 第一次调用，记录时间
        if self.last_time is None:
            self.last_time = current_time
            dt = 0.0
        else:
            dt = current_time - self.last_time
            self.last_time = current_time
        if dt <= 0:
            # 避免除0
            dt = 1e-9

        # 积分项
        self.integral += error * dt
        # 积分限幅
        if self.integral > self.integral_limit:
            self.integral = self.integral_limit
        elif self.integral < -self.integral_limit:
            self.integral = -self.integral_limit

        # 微分项
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        # PID输出
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        # 输出限幅
        if output > self.output_limit:
            output = self.output_limit
        elif output < -self.output_limit:
            output = -self.output_limit
        return output