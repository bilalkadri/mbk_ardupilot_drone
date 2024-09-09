import matplotlib.pyplot as plt
import pickle

def read_data():
    f = open('/home/ugv/rtab_ws/src/mbk_ardupilot_drone/script/data.pickle', 'rb')
    data = pickle.load(f)
    f.close()

    return data

errors_dict = read_data()

blue_water_reservoir_pid_x_errors = errors_dict['blue_x_errors']
blue_water_reservoir_pid_y_errors = errors_dict['blue_y_errors']
blue_water_reservoir_pid_z_errors = errors_dict['blue_z_errors']

red_water_discharge_pid_x_errors = errors_dict['red_x_errors']
red_water_discharge_pid_y_errors = errors_dict['red_y_errors']
red_water_discharge_pid_z_errors = errors_dict['red_z_errors']

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

# Plot blue_errors on the first subplot
ax1.plot(blue_water_reservoir_pid_x_errors, color='blue')
ax1.set_title('Blue Water Reservoir X Errors')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Error')
ax1.grid(True)

ax2.plot(blue_water_reservoir_pid_y_errors, color='blue')
ax2.set_title('Blue Water Reservoir Y Errors')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Error')
ax2.grid(True)

ax3.plot(blue_water_reservoir_pid_z_errors, color='blue')
ax3.set_title('Blue Water Reservoir Z Errors')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Error')
ax3.grid(True)

# Adjust layout and display the plot
plt.tight_layout()
plt.show(block=False)

# Create a figure with three subplots
fig2, (rax1, rax2, rax3) = plt.subplots(3, 1, figsize=(10, 8))

rax1.plot(red_water_discharge_pid_x_errors, color='red')
rax1.set_title('Red Water Discharge X Errors')
rax1.set_xlabel('Time (s)')
rax1.set_ylabel('Error')
rax1.grid(True)

rax2.plot(red_water_discharge_pid_y_errors, color='red')
rax2.set_title('Red Water Discharge Y Errors')
rax2.set_xlabel('Time (s)')
rax2.set_ylabel('Error')
rax2.grid(True)

rax3.plot(red_water_discharge_pid_z_errors, color='red')
rax3.set_title('Red Water Discharge Z Errors')
rax3.set_xlabel('Time (s)')
rax3.set_ylabel('Error')
rax3.grid(True)

# Adjust layout and display the plot
plt.tight_layout()
plt.show(block=False)

# Keep the plots open
input("Press Enter to close the plots...")