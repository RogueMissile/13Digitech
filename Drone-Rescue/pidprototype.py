import time
import matplotlib.pyplot as plt

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.last_error = 0
        self.integral = 0
        self.last_time = time.time()

    def update(self, process_variable):
        current_time = time.time()
        dt = current_time - self.last_time

        error = self.setpoint - process_variable

        proportional = self.Kp * error
        self.integral += self.Ki * error * dt
        derivative = self.Kd * (error - self.last_error) / dt if dt > 0 else 0

        output = proportional + self.integral + derivative

        self.last_error = error
        self.last_time = current_time
        return output

pid_controller = PIDController(Kp=15, Ki=12, Kd=0.001, setpoint=90)

current_value = 50
values = []
outputs = []
timestamps = []

plt.ion()
fig, ax = plt.subplots(figsize=(10, 5))
line1, = ax.plot([], [], label="Process Variable")
line2, = ax.plot([], [], label="Control Output")
ax.set_xlabel("Time (seconds)")
ax.set_ylabel("Value")
ax.set_title("PID Controller Test")
ax.legend()
ax.grid()

start_time = time.time()

for _ in range(190):
    control_output = pid_controller.update(current_value)
    current_value += (control_output - current_value) * 0.1

    values.append(current_value)
    outputs.append(control_output)
    timestamps.append(time.time() - start_time)

    line1.set_xdata(timestamps)
    line1.set_ydata(values)
    line2.set_xdata(timestamps)
    line2.set_ydata(outputs)
    
    ax.relim()
    ax.autoscale_view()
    plt.draw()
    plt.pause(0.1)

plt.ioff() 
plt.show()