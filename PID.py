class PID:
    def __init__(self, kp: float, ki: float, kd: float, setpoint: float = 0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint

        # Internal variables to store previous error and integral of error
        self._previous_error = 0.0
        self._integral = 0.0

    def update(self, current_value: float, dt: float) -> float:
        """
        Calculate the PID control signal.

        :param current_value: The current value of the process variable.
        :param dt: Time interval since the last update.
        :return: The control signal.
        """
        # Calculate the error
        error = self.setpoint - current_value

        # Proportional term
        p_term = self.kp * error

        # Integral term
        self._integral += error * dt
        i_term = self.ki * self._integral

        # Derivative term
        derivative = (error - self._previous_error) / dt if dt > 0 else 0.0
        d_term = self.kd * derivative

        # Store current error as previous for next iteration
        self._previous_error = error

        # Calculate total output
        output = p_term + i_term + d_term
        return output

    def reset(self):
        """
        Reset the PID controller.
        """
        self._previous_error = 0.0
        self._integral = 0.0
