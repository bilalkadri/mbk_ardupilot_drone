import numpy as np
import matplotlib.pyplot as plt
import control as ctl


# Define the first-order transfer function G(s) = 1/(Ts + 1)
T = 1  # Time constant of the first-order system
G = ctl.TransferFunction([1], [T, 1])

# Define the PID controller gains
Kp = 1.0  # Proportional gain
Ki = 0.5  # Integral gain
Kd = 0.05  # Derivative gain
tau = 0.01  # Small time constant to make the system proper


# PID transfer function with derivative filtering:
# Proportional + Integral + Filtered Derivative
P = ctl.TransferFunction([Kp], [1])
I = ctl.TransferFunction([Ki], [1, 0])
D = ctl.TransferFunction([Kd, 0], [tau, 1])  # Derivative with filter

# Combined PID controller
pid = P + I + D

# Closed-loop system with feedback
system = ctl.feedback(pid * G, 1)

# Closed-loop system with feedback
system = ctl.feedback(pid * G, 1)

# Define time vector and reference input (step input)
t = np.linspace(0, 10, 1000)  # Time vector
t, y = ctl.step_response(system, T=t)  # System output (response to step input)

# Control signal (input to the system) is the PID controller response
control_signal = ctl.forced_response(pid, T=t, U=y)[1]

# Plot the results
plt.figure(figsize=(10, 6))

# Plot the reference signal (step input)
plt.plot(t, np.ones_like(t), 'g--', label='Reference Input (Step)')

# Plot the system output
plt.plot(t, y, 'b-', label='System Output')

# Plot the control signal
plt.plot(t, control_signal, 'r-', label='Control Signal (PID output)')

# Customize plot
plt.title('PID Controller with Low-Pass Filter for a First Order System')
plt.xlabel('Time (seconds)')
plt.ylabel('Amplitude')
plt.legend(loc='best')
plt.grid(True)

# Show the plot
plt.show()
