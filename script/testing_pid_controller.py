from scipy.optimize import minimize
from pid_controller_mbk import pid_controller


# Objective function
def objective_function(x):
    return x[0]**2 + x[1]**2

# Initial guess
x0 = [1, 1]

# Minimize the objective function
result = minimize(objective_function, x0)

Y_Error_list=[0.1,0.2,0.3,0.4,0.5,0.6,0.7]
Y_Error_list_for_computation=Y_Error_list[-3:]

# print(Y_Error_list_for_computation)
# print(Y_Error_list_for_computation[0])

# Print the result
print(result)

# controller_z = pid_controller(Kp, Ki, Kd, limit)
test_controller=pid_controller(1,2,3,4)
test_controller.set_Kp(2)


