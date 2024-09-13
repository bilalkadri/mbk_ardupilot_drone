
import numpy as np
import matplotlib.pyplot as plt
import control as ctl

from fuzzy_pid_controller_mbk import fuzzy_pid_controller


#Instantiatiung thr fuzzy controller
controller_x = fuzzy_pid_controller(1, 0.5, 0.05, 0.01)



# System and PID parameters
T = 10  # Time constant of the first-order system
Kp = 1.0  # Proportional gain
Ki = 0.5  # Integral gain
Kd = 0.05  # Derivative gain
tau = 0.01  # Derivative filter time constant
u_max = 20.0  # Control signal maximum (anti-windup)
response_output = []  # Store the output response over time

# Define the first-order transfer function G(s) = 1/(Ts + 1)
G = ctl.TransferFunction([1], [T,T, 1])

# Simulation parameters
dt = 0.01  # Time step
t = np.arange(0, 100, dt)  # Time vector
reference = np.ones_like(t)  # Step reference input

# Initialize PID variables
integral = 0
prev_error = 0
prev_output = 0
control_signal_crisp_PID = []
control_signal_fuzzy_PID = []

# Manually simulate the PID control loop with anti-windup
for i in range(len(t)):
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

    # Fuzzy PID control signal 
    # u_fuzzy_PID=controller_x.set_current_error(error) 
    u_fuzzy_PID=0
    # print(u_fuzzy_PID)

    # Apply control signal saturation to prevent windup
    if u_crisp_PID > u_max:
        u_crisp_PID = u_max
        # Disable integral action if the control signal is saturated
        integral -= error * dt  # Undo the integration that caused windup
    elif u_crisp_PID < -u_max:
        u_crisp_PID = -u_max
        integral -= error * dt  # Undo the integration that caused windup

    control_signal_crisp_PID.append(u_crisp_PID)
    control_signal_fuzzy_PID.append(u_fuzzy_PID)

    # Update the system response for the current time step
    # Pass incremental time step [i*dt, (i+1)*dt] and control signal
    time_increment = [i * dt, (i + 1) * dt]  # Define the time range for this step
    input_signal = [prev_output, u_crisp_PID]  # Input at time step [previous, current]


    # Update the system response (discrete first-order system)
    time,response= ctl.forced_response(G, T=time_increment, U=input_signal)
    prev_output = response[-1]
    # Save the current system response (last output)
    response_output.append(prev_output)
#    _, prev_output, _ = ctl.forced_response(G, T=[0, dt], U=[prev_output, u_fuzzy_PID])

# Convert control signal to numpy array for plotting
control_signal_crisp_PID = np.array(control_signal_crisp_PID)
control_signal_fuzzy_PID = np.array(control_signal_fuzzy_PID)


# Plot the results
plt.figure(figsize=(10, 6))

# Plot the reference signal (step input)
plt.plot(t, reference, 'g--', label='Reference Input (Step)')

# Plot the system output in blue
plt.plot(t, response_output, 'b-', label='System Output (Response)', color='blue')



# Plot the system output
plt.plot(t, control_signal_crisp_PID, 'r-', label='Control Signal (Crisp PID output)')
# plt.plot(t, control_signal_fuzzy_PID, 'k-', label='Control Signal (Fuzzy PID output)')





# Customize plot
plt.title('PID Controller with Integral Windup Prevention')
plt.xlabel('Time (seconds)')
plt.ylabel('Amplitude')
plt.legend(loc='best')
plt.grid(True)

# Show the plot
plt.show()
