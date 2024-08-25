from scipy.optimize import minimize

History_of_x=[]

# Objective function
def objective_function(x):
    global History_of_x
    History_of_x.append(x)
    return x[0]**2 + x[1]**2

# Initial guess
x0 = [5, 5]

# Minimize the objective function
result = minimize(objective_function, x0)

# Y_Error_list=[0.1,0.2,0.3,0.4,0.5,0.6,0.7]
# Y_Error_list_for_computation=Y_Error_list[-3:]

# print(Y_Error_list_for_computation)
# print(Y_Error_list_for_computation[0])

# Print the result
print(result)
result.x[0]
result.x[1]

print(result.x[0])
# print(History_of_x)
# controller_z = pid_controller(Kp, Ki, Kd, limit)

