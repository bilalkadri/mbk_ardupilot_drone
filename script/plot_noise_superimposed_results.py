import matplotlib.pyplot as plt
import pickle

file_path = '/home/ugv/rtab_ws/src/mbk_ardupilot_drone/experiment_results/'
file_case1 = file_path + 'pid_LW_LowNoise.pickle'
file_case2 = file_path + 'pid_LW_MedNoise.pickle'
file_case3 = file_path + 'pid_LW_HighNoise.pickle'
file_case4 = file_path + 'pid_LW_noNoise.pickle'

logfile = file_case1


def read_data(file):
    f = open(file, 'rb')
    data = pickle.load(f)
    f.close()

    return data

errors_dict_case1 = read_data(file_case1)
errors_dict_case2 = read_data(file_case2)
errors_dict_case3 = read_data(file_case3)
errors_dict_case4 = read_data(file_case4)

# Create a figure with three subplots
fig1, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))

min_x = min(min(errors_dict_case1['blue_xy_t']),min(errors_dict_case2['blue_xy_t']),min(errors_dict_case3['blue_xy_t']),min(errors_dict_case4['blue_xy_t']),
            min(errors_dict_case1['blue_z_t']),min(errors_dict_case2['blue_z_t']),min(errors_dict_case3['blue_z_t']),min(errors_dict_case4['blue_z_t']))
max_x = max(max(errors_dict_case1['blue_xy_t']),max(errors_dict_case2['blue_xy_t']),max(errors_dict_case3['blue_xy_t']),max(errors_dict_case4['blue_xy_t']),
            max(errors_dict_case1['blue_z_t']),max(errors_dict_case2['blue_z_t']),max(errors_dict_case3['blue_z_t']),max(errors_dict_case4['blue_z_t']))

# Plot blue_errors on the first subplot
ax1.plot(errors_dict_case1['blue_xy_t'], errors_dict_case1['blue_x_errors'], label='Low Wind Low Noise', color='green')
ax1.plot(errors_dict_case2['blue_xy_t'], errors_dict_case2['blue_x_errors'], label='Low Wind Med Noise', color='blue')
ax1.plot(errors_dict_case3['blue_xy_t'], errors_dict_case3['blue_x_errors'], label='Low Wind High Noise', color='red')
ax1.plot(errors_dict_case4['blue_xy_t'], errors_dict_case4['blue_x_errors'], label='Low Wind No Noise', color='orange')
ax1.set_xlim(min_x, max_x)
ax1.set_title('Blue Water Reservoir X Errors')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Error')
ax1.legend()
ax1.grid(True)

ax2.plot(errors_dict_case1['blue_xy_t'], errors_dict_case1['blue_y_errors'], label='Low Wind Low Noise', color='green')
ax2.plot(errors_dict_case2['blue_xy_t'], errors_dict_case2['blue_y_errors'], label='Low Wind Med Noise', color='blue')
ax2.plot(errors_dict_case3['blue_xy_t'], errors_dict_case3['blue_y_errors'], label='Low Wind High Noise', color='red')
ax2.plot(errors_dict_case4['blue_xy_t'], errors_dict_case4['blue_y_errors'], label='Low Wind No Noise', color='orange')
ax2.set_xlim(min_x, max_x)
ax2.set_title('Blue Water Reservoir Y Errors')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Error')
ax2.legend()
ax2.grid(True)

ax3.plot(errors_dict_case1['blue_z_t'], errors_dict_case1['blue_z_errors'], label='Low Wind Low Noise', color='green')
ax3.plot(errors_dict_case2['blue_z_t'], errors_dict_case2['blue_z_errors'], label='Low Wind Med Noise', color='blue')
ax3.plot(errors_dict_case3['blue_z_t'], errors_dict_case3['blue_z_errors'], label='Low Wind High Noise', color='red')
ax3.plot(errors_dict_case4['blue_z_t'], errors_dict_case4['blue_z_errors'], label='Low Wind No Noise', color='orange')
ax3.set_xlim(min_x, max_x)
ax3.set_title('Blue Water Reservoir Z Errors')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Error')
ax3.legend()
ax3.grid(True)

# Adjust layout and display the plot
plt.tight_layout()
plt.show(block=False)

# Create a figure with three subplots
fig2, (rax1, rax2, rax3) = plt.subplots(3, 1, figsize=(10, 8))

min_xx = min(min(errors_dict_case1['red_xy_t']),min(errors_dict_case2['red_xy_t']),min(errors_dict_case3['red_xy_t']),min(errors_dict_case4['red_xy_t']),
            min(errors_dict_case1['red_z_t']),min(errors_dict_case2['red_z_t']),min(errors_dict_case3['red_z_t']), min(errors_dict_case4['red_z_t']))
max_xx = max(max(errors_dict_case1['red_xy_t']),max(errors_dict_case2['red_xy_t']),max(errors_dict_case3['red_xy_t']),max(errors_dict_case4['red_xy_t']),
            max(errors_dict_case1['red_z_t']),max(errors_dict_case2['red_z_t']),max(errors_dict_case3['red_z_t']),max(errors_dict_case4['red_z_t']))

rax1.plot(errors_dict_case1['red_xy_t'], errors_dict_case1['red_x_errors'], label='Low Wind Low Noise', color='green')
rax1.plot(errors_dict_case2['red_xy_t'], errors_dict_case2['red_x_errors'], label='Low Wind Med Noise', color='blue')
rax1.plot(errors_dict_case3['red_xy_t'], errors_dict_case3['red_x_errors'], label='Low Wind High Noise', color='red')
rax1.plot(errors_dict_case4['red_xy_t'], errors_dict_case4['red_x_errors'], label='Low Wind No Noise', color='orange')
rax1.set_xlim(min_xx, max_xx)
rax1.set_title('Red Water Discharge X Errors')
rax1.set_xlabel('Time (s)')
rax1.set_ylabel('Error')
rax1.legend()
rax1.grid(True)

rax2.plot(errors_dict_case1['red_xy_t'], errors_dict_case1['red_y_errors'], label='Low Wind Low Noise', color='green')
rax2.plot(errors_dict_case2['red_xy_t'], errors_dict_case2['red_y_errors'], label='Low Wind Med Noise', color='blue')
rax2.plot(errors_dict_case3['red_xy_t'], errors_dict_case3['red_y_errors'], label='Low Wind High Noise', color='red')
rax2.plot(errors_dict_case4['red_xy_t'], errors_dict_case4['red_y_errors'], label='Low Wind No Noise', color='orange')
rax2.set_xlim(min_xx, max_xx)
rax2.set_title('Red Water Discharge Y Errors')
rax2.set_xlabel('Time (s)')
rax2.set_ylabel('Error')
rax2.legend()
rax2.grid(True)

rax3.plot(errors_dict_case1['red_z_t'], errors_dict_case1['red_z_errors'], label='Low Wind Low Noise', color='green')
rax3.plot(errors_dict_case2['red_z_t'], errors_dict_case2['red_z_errors'], label='Low Wind Med Noise', color='blue')
rax3.plot(errors_dict_case3['red_z_t'], errors_dict_case3['red_z_errors'], label='Low Wind High Noise', color='red')
rax3.plot(errors_dict_case4['red_z_t'], errors_dict_case4['red_z_errors'], label='Low Wind No Noise', color='orange')
rax3.set_xlim(min_xx, max_xx)
rax3.set_title('Red Water Discharge Z Errors')
rax3.set_xlabel('Time (s)')
rax3.set_ylabel('Error')
rax3.legend()
rax3.grid(True)

# Adjust layout and display the plot
plt.tight_layout()
plt.show(block=False)

# Keep the plots open
input("Press Enter to close the plots...")