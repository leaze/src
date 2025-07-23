import time

class PIDController:
    def __init__(self, kp, ki, kd, min=0.0, max=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = min
        self.max = max
        self.reset()
        # 状态变量
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = None
        self.last_error = 0.0
        self.last_derivative = 0.0

        # 滤波器参数
        self.derivative_filter = 0.0
        self.filter_constant = 0.1

    def reset(self):
        """重置PID控制器状态"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = None
        self.last_error = 0.0
        self.last_derivative = 0.0

    def update(self, error, derivative=None, current_time=None):
        """
        更新PID控制器并计算控制输出
        Args:
            error: 当前位置误差 (目标位置 - 当前位置)
            derivative: 可选，当前速度微分（用于微分项）
            current_time: 可选，当前时间戳
        Returns:
            控制输出值(在min和max之间)
        """
        # 获取当前时间（如果未提供）
        if current_time is None:
            current_time = time.time()

        # 计算时间步长
        dt = self._get_time_delta(current_time)

        # 计算微分项（使用提供的导数或自行计算）
        if derivative is None:
            derivative = self._calculate_derivative(error, dt)
        else:
            # 对提供的导数进行滤波平滑
            derivative = self._filter_derivative(derivative)

        # 计算积分项（带抗饱和逻辑）
        self._update_integral(error, dt)

        # PID输出计算
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # 保存状态用于下次计算
        self.last_error = error
        self.last_derivative = derivative

        # 输出限幅
        return max(self.min, min(output, self.max))

    def _get_time_delta(self, current_time):
        """计算时间步长"""
        if self.last_time is None:
            # 第一次调用，使用预设的小时间步长
            self.last_time = current_time
            return 0.01  # 默认10ms

        dt = current_time - self.last_time
        self.last_time = current_time

        # 处理无效时间步长
        if dt <= 0:
            dt = 0.001  # 防止除零或负时间步

        return dt

    def _calculate_derivative(self, error, dt):
        """计算位置误差的微分（速度）"""
        derivative = (error - self.last_error) / dt if self.last_error != error else 0.0

        # 对计算的导数进行滤波
        return self._filter_derivative(derivative)

    def _filter_derivative(self, derivative):
        """低通滤波微分项以减小噪声影响"""
        if self.filter_constant > 0 and self.filter_constant < 1.0:
            filtered = (1 - self.filter_constant) * self.last_derivative + self.filter_constant * derivative
            return filtered
        return derivative

    def _update_integral(self, error, dt):
        """更新积分项并处理积分饱和"""
        # 积分项更新
        self.integral += error * dt
        
        # 积分抗饱和逻辑
        if abs(self.ki) > 1e-6:  # 避免除以零
            integral_term = self.ki * self.integral
            if integral_term > self.max:
                self.integral = self.max / self.ki
            elif integral_term < self.min:
                self.integral = self.min / self.ki
        else:
            # 当 ki 为零时，不需要处理积分饱和
            pass