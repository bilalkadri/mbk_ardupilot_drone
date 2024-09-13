#In this code the transfer function 'G' is defined within the code
#IN our problem the plant should be defined outside this code so that we 
#can test the performance of our controller with quadcopter

# We cannot use this code to test our scheme


import control as ctl
import numpy as np
import matplotlib.pyplot as plt

# System and PID parameters
T = 10  # Time constant of the first-order system
Kp = 1.0  # Proportional gain
Ki = 0.5  # Integral gain
Kd = 0.05  # Derivative gain
tau = 0.01  # Derivative filter time constant
u_max = 20.0  # Control signal maximum (anti-windup)
response_output = []  # Store the output response over time

# Define the first-order transfer function G(s) = 1/(Ts + 1)
G = ctl.TransferFunction([1], [T, 1])

# Simulation parameters
dt = 0.01  # Time step
t = np.arange(0, 100, dt)  # Time vector
reference = np.ones_like(t)  # Step reference input

# Initialize PID variables
integral = 0
prev_error = 0
prev_output = 0
control_signal_crisp_PID = []

# Initialize time and response storage for the simulation
response_output = []
control_signal_crisp_PID = []

# Manually simulate the PID control loop with anti-windup
for i in range(len(t)):
    # Calculate error
    error = reference[i] - prev_output

    # Proportional term
    P = Kp * error

    # Integral term (with windup prevention)
    integral += error * dt
    I = Ki * integral

    # Derivative term (with filtering)
    D = Kd * (error - prev_error) / dt / (tau / dt + 1)
    prev_error = error

    # PID control signal
    u_crisp_PID = P + I + D

    # Apply control signal saturation to prevent windup
    if u_crisp_PID > u_max:
        u_crisp_PID = u_max
        integral -= error * dt  # Undo the integration that caused windup
    elif u_crisp_PID < -u_max:
        u_crisp_PID = -u_max
        integral -= error * dt  # Undo the integration that caused windup

    # Append control signal for plotting later
    control_signal_crisp_PID.append(u_crisp_PID)

    # Update the system response for the current time step
    time_increment = np.array([i * dt, (i + 1) * dt])  # Define time range
    input_signal = [prev_output, u_crisp_PID]  # Input at previous and current time

    # Update the system response (discrete first-order system)
    # _, response, _ = ctl.forced_response(G, T=time_increment, U=input_signal)
    

    time,response= ctl.forced_response(G, T=time_increment, U=input_signal)
    
    
    # Update the previous output for the next iteration
    prev_output = response[-1]

    # Save the current system response (last output)
    response_output.append(prev_output)

# Plot the response and control signals
plt.figure()
plt.plot(t, response_output, label='System Response')
plt.plot(t, reference, label='Reference Signal', linestyle='--')
plt.plot(t, control_signal_crisp_PID, label='Control Signal (PID)', linestyle=':')
plt.xlabel('Time (s)')
plt.ylabel('Response / Control Signal')
plt.legend()
plt.title('PID Control of First-Order System')
plt.grid(True)
plt.show()
