import numpy as np
import skfuzzy as fuzz
import matplotlib.pyplot as plt

# Define the universe of discourse
x_Ki = np.linspace(0, 2.5, 500)

# Define triangular membership functions for K_i
K_i_pvs = fuzz.trimf(x_Ki, [0, 0, 0.5])
K_i_ps  = fuzz.trimf(x_Ki, [0, 0.5, 1])
K_i_pms = fuzz.trimf(x_Ki, [0.5, 1, 1.5])
K_i_pm  = fuzz.trimf(x_Ki, [1, 1.25, 1.5])
K_i_pml = fuzz.trimf(x_Ki, [1, 1.5, 2])
K_i_pl  = fuzz.trimf(x_Ki, [1.5, 2, 2.5])
K_i_pvl = fuzz.trimf(x_Ki, [2, 2.5, 2.5])

# Plotting the membership functions
plt.figure(figsize=(8, 6))

plt.plot(x_Ki, K_i_pvs, 'b', label='pvs')
plt.plot(x_Ki, K_i_ps, 'r', label='ps')
plt.plot(x_Ki, K_i_pms, 'orange', label='pms')
plt.plot(x_Ki, K_i_pm, 'purple', label='pm')
plt.plot(x_Ki, K_i_pml, 'g', label='pml')
plt.plot(x_Ki, K_i_pl, 'cyan', label='pl')
plt.plot(x_Ki, K_i_pvl, 'brown', label='pvl')

plt.title('Triangular Membership Functions for $K_i$')
plt.xlabel('$K_i$')
plt.ylabel('Degree of Membership')
plt.legend(loc='upper right')
plt.show()
