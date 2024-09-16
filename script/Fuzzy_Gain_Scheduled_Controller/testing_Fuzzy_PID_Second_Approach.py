from fuzzy_pid_controller_mbk_updated import fuzzy_pid_controller
from pid_controller_mbk import pid_controller
import numpy as np
import matplotlib.pyplot as plt

from scipy.integrate import odeint
import numpy as np
import matplotlib.pyplot as plt


controller_x = fuzzy_pid_controller(0.6, .2, 0.1, 20)
# controller_x=pid_controller(0.6, 0.2, 0.1, 20)

# print("I am here")

def system(t, temp, Tq):
    epsilon = 1
    tau = 4
    Tf = 300
    Q = 2
    dTdt = 1/(tau*(1+epsilon)) * (Tf-temp) + Q/(1+epsilon)*(Tq-temp)
    return dTdt


# number of steps
n = 2000

time_prev = 0
y0 = 300
deltat = 0.01
y_sol = [y0]
t_sol = [time_prev]

# Tq is chosen as a manipulated variable
Tq = 320,
control_signal_list=[0]

q_sol = [Tq[0]]
setpoint = 310
setpoint_list=[setpoint]*n
integral = 0

for i in range(1, n):
    time = i * deltat
    tspan = np.linspace(time_prev, time, 10)
    control_signal = controller_x.set_current_error(setpoint-y_sol[-1]),
   
    yi = odeint(system,y_sol[-1], tspan, args=(Tq,), tfirst=True)
    t_sol.append(time)
   
    y_sol.append(yi[-1][0])
    q_sol.append(Tq[0])
    control_signal_list.append(control_signal[0])
   
    time_prev = time

# print(y_sol)
plt.subplot(2, 1, 1) 
plt.plot(t_sol, y_sol,color='blue')
plt.xlabel('Time')
plt.ylabel('Temperature')
plt.plot(t_sol, setpoint_list,color='red')
# Display the plot

plt.subplot(2, 1, 2) 
print(len(control_signal_list))
plt.plot(t_sol, control_signal_list,color='blue')
plt.xlabel('Time')
plt.ylabel('Control Signal')
plt.show()