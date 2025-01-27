import matplotlib.pyplot as plt
import pickle

file_path = '/home/ugv/rtab_ws/src/mbk_ardupilot_drone/experiment_results/'
file_name = 'flc_LW_noNoise.pickle'

logfile = file_path + file_name


def read_data(file):
    f = open(file, 'rb')
    data = pickle.load(f)
    f.close()

    return data

errors_dict = read_data(logfile)

blue_water_reservoir_pid_x_errors = errors_dict['blue_x_errors']
blue_water_reservoir_pid_y_errors = errors_dict['blue_y_errors']
blue_water_reservoir_pid_z_errors = errors_dict['blue_z_errors']
blue_water_reservoir_sp_x = errors_dict['blue_x_sp']
blue_water_reservoir_sp_y = errors_dict['blue_y_sp']
blue_water_reservoir_sp_z = errors_dict['blue_z_sp']
blue_water_reservoir_xy_t = errors_dict['blue_xy_t']
blue_water_reservoir_z_t = errors_dict['blue_z_t']

red_water_discharge_pid_x_errors = errors_dict['red_x_errors']
red_water_discharge_pid_y_errors = errors_dict['red_y_errors']
red_water_discharge_pid_z_errors = errors_dict['red_z_errors']
red_water_discharge_sp_x = errors_dict['red_x_sp']
red_water_discharge_sp_y = errors_dict['red_y_sp']
red_water_discharge_sp_z = errors_dict['red_z_sp']
red_water_discharge_xy_t = errors_dict['red_xy_t']
red_water_discharge_z_t = errors_dict['red_z_t']

blue_t_min = min(min(blue_water_reservoir_xy_t),min(blue_water_reservoir_z_t))
blue_t_max = max(max(blue_water_reservoir_xy_t),max(blue_water_reservoir_z_t))

red_t_min = min(min(red_water_discharge_xy_t),min(red_water_discharge_z_t))
red_t_max = max(max(red_water_discharge_xy_t),max(red_water_discharge_z_t))

####### To plot all errors on a separate figure #######

# plt.figure()
# plt.plot(blue_water_reservoir_pid_x_errors, color='blue')
# plt.title('Blue Water Reservoir X Errors')
# plt.xlabel('Time (s)')
# plt.ylabel('Error')
# plt.grid(True)
# plt.show(block=False)  

# plt.figure()
# plt.plot(blue_water_reservoir_pid_y_errors, color='blue')
# plt.title('Blue Water Reservoir Y Errors')
# plt.xlabel('Time (s)')
# plt.ylabel('Error')
# plt.grid(True)
# plt.show(block=False)  

# plt.figure()
# plt.plot(blue_water_reservoir_pid_z_errors, color='blue')
# plt.title('Blue Water Reservoir Z Errors')
# plt.xlabel('Time (s)')
# plt.ylabel('Error')
# plt.grid(True)
# plt.show(block=False)  

# plt.figure()
# plt.plot(red_water_discharge_pid_x_errors, color='red')
# plt.title('Red Water Discharge X Errors')
# plt.xlabel('Time (s)')
# plt.ylabel('Error')
# plt.grid(True)
# plt.show(block=False)  

# plt.figure()
# plt.plot(red_water_discharge_pid_y_errors, color='red')
# plt.title('Red Water Discharge X Errors')
# plt.xlabel('Time (s)')
# plt.ylabel('Error')
# plt.grid(True)
# plt.show(block=False)  

# plt.figure()
# plt.plot(red_water_discharge_pid_z_errors, color='red')
# plt.title('Red Water Discharge X Errors')
# plt.xlabel('Time (s)')
# plt.ylabel('Error')
# plt.grid(True)
# plt.show(block=False)  

####### Plot errors on 2 subplots #######

# Create a figure with three subplots
fig1, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))

ax1.plot(blue_water_reservoir_xy_t, blue_water_reservoir_pid_x_errors, color='blue')
ax1.set_xlim(blue_t_min, blue_t_max)
ax1.set_title('Blue Water Reservoir X Errors')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Error')
ax1.grid(True)

ax2.plot(blue_water_reservoir_xy_t, blue_water_reservoir_pid_y_errors, color='blue')
ax2.set_xlim(blue_t_min, blue_t_max)
ax2.set_title('Blue Water Reservoir Y Errors')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Error')
ax2.grid(True)

ax3.plot(blue_water_reservoir_z_t, blue_water_reservoir_pid_z_errors, color='blue')
ax3.set_xlim(blue_t_min, blue_t_max)
ax3.set_title('Blue Water Reservoir Z Errors')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Error')
ax3.grid(True)

# Adjust layout and display the plot
plt.tight_layout()
plt.show(block=False)

# Create a figure with three subplots
fig2, (rax1, rax2, rax3) = plt.subplots(3, 1, figsize=(10, 8))

rax1.plot(red_water_discharge_xy_t, red_water_discharge_pid_x_errors, color='red')
rax1.set_xlim(red_t_min, red_t_max)
rax1.set_title('Red Water Discharge X Errors')
rax1.set_xlabel('Time (s)')
rax1.set_ylabel('Error')
rax1.grid(True)

rax2.plot(red_water_discharge_xy_t, red_water_discharge_pid_y_errors, color='red')
rax2.set_xlim(red_t_min, red_t_max)
rax2.set_title('Red Water Discharge Y Errors')
rax2.set_xlabel('Time (s)')
rax2.set_ylabel('Error')
rax2.grid(True)

rax3.plot(red_water_discharge_z_t, red_water_discharge_pid_z_errors, color='red')
rax3.set_xlim(red_t_min, red_t_max)
rax3.set_title('Red Water Discharge Z Errors')
rax3.set_xlabel('Time (s)')
rax3.set_ylabel('Error')
rax3.grid(True)

# Adjust layout and display the plot
plt.tight_layout()
plt.show(block=False)

# Keep the plots open
input("Press Enter to close the plots...")