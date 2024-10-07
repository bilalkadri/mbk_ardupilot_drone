
from fuzzy_pid_controller_mbk_updated import fuzzy_pid_controller
from fuzzy_pi_controller_mbk import fuzzy_pi_controller
from pid_controller_mbk import pid_controller

import numpy as np
import matplotlib.pyplot as plt

from scipy.integrate import odeint
import numpy as np
import matplotlib.pyplot as plt
import control as ctl

# controller_x = fuzzy_pid_controller(0.6, .2, 0.1, 20)
controller_x = fuzzy_pi_controller(5, .9,  20)
# controller_x=pid_controller(0.9, 0.9, 0.1, 20)

# print("I am here")

def system1(t, temp, Tq):
    epsilon = 1
    tau = 4
    Tf = 300
    Q = 2
    dTdt = 1/(tau*(1+epsilon)) * (Tf-temp) + Q/(1+epsilon)*(Tq-temp)
  
    return dTdt

def system2(t, y, u):
    
    tau = 10
    #First Order Plant Y(s)/U(s)=1/(Tau*s+1)
    dYdt=(1/tau)*(u-y)
    return dYdt

# number of steps
n = 5000

time_prev = 0

deltat = 0.01
t_sol = [time_prev]

# System 1, Tq is chosen as a manipulated variable
# u = 320,  # This is Tq
# q_sol = [u[0]]
# setpoint = 310
# y0_system_1 = 300
# y_sol = [y0_system_1]
# setpoint_list=[setpoint]*n

# #System 2, u is chosen as the manipulated variable (control variable)
u = 0.5,
q_sol = [u[0]]
setpoint = 1
y0_system_2=0.5
y_sol=[y0_system_2] 
setpoint_list=[setpoint]*n

integral = 0
control_signal_list=[0]

for i in range(1, n):
    time = i * deltat
    tspan = np.linspace(time_prev, time, 10)
    control_signal = controller_x.set_current_error(setpoint-y_sol[-1]),
    # print(setpoint-y_sol[-1])
    # yi = odeint(system1,y_sol[-1], tspan, args=control_signal, tfirst=True)
    yi = odeint(system2,y_sol[-1], tspan, args=control_signal, tfirst=True)

    t_sol.append(time)
   
    y_sol.append(yi[-1][0])
    # q_sol.append(Tq[0])  #For System 1
    q_sol.append(u[0])     # For System 2
    
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