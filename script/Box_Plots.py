
import matplotlib.pyplot as plt
import pickle
import numpy as np

# file_path= 'C:\Users\bilal\Dropbox\My Research Publications\Journal\Submitted for peer review\JP 01 (PeerJ, MBK, Drone Architecture)\mbk_ardupilot_drone\experiment_results'
file_path= 'C:\\Users\\bilal\\Dropbox\\My Research Publications\\Journal\\Submitted for peer review\\JP 01 (PeerJ, MBK, Drone Architecture)\\mbk_ardupilot_drone\\experiment_results\\'
#file_path = '/home/ugv/rtab_ws/src/mbk_ardupilot_drone/experiment_results/'
file_case1 = file_path + 'pid_LW_noNoise.pickle'
file_case2 = file_path + 'pid_MW_noNoise.pickle'
file_case3 = file_path + 'pid_HW_noNoise.pickle'

logfile = file_case1


def read_data(file):
    f = open(file, 'rb')
    data = pickle.load(f)
    f.close()

    return data
# Prepare data for boxplots

errors_dict_case1 = read_data(file_case1)
errors_dict_case2 = read_data(file_case2)
errors_dict_case3 = read_data(file_case3)


def prepare_error_data(case1, case2, case3, key):
    return [
        case1[key],  # Low wind
        case2[key],  # Medium wind
        case3[key]   # High wind
    ]

# Titles and labels
labels = ['Low Wind', 'Med Wind', 'High Wind']

# === BLUE TARGET BOX PLOTS ===
fig_blue, axes_blue = plt.subplots(1, 3, figsize=(15, 5))
fig_blue.suptitle('Blue Water Reservoir - Positional Error Distribution', fontsize=14)

blue_x = prepare_error_data(errors_dict_case1, errors_dict_case2, errors_dict_case3, 'blue_x_errors')
blue_y = prepare_error_data(errors_dict_case1, errors_dict_case2, errors_dict_case3, 'blue_y_errors')
blue_z = prepare_error_data(errors_dict_case1, errors_dict_case2, errors_dict_case3, 'blue_z_errors')

axes_blue[0].boxplot(blue_x, labels=labels)
axes_blue[0].set_title('X Errors')
axes_blue[0].set_ylabel('Error')

axes_blue[1].boxplot(blue_y, labels=labels)
axes_blue[1].set_title('Y Errors')

axes_blue[2].boxplot(blue_z, labels=labels)
axes_blue[2].set_title('Z Errors')

plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show(block=False)

# === RED TARGET BOX PLOTS ===
fig_red, axes_red = plt.subplots(1, 3, figsize=(15, 5))
fig_red.suptitle('Red Water Discharge - Positional Error Distribution', fontsize=14)

red_x = prepare_error_data(errors_dict_case1, errors_dict_case2, errors_dict_case3, 'red_x_errors')
red_y = prepare_error_data(errors_dict_case1, errors_dict_case2, errors_dict_case3, 'red_y_errors')
red_z = prepare_error_data(errors_dict_case1, errors_dict_case2, errors_dict_case3, 'red_z_errors')

axes_red[0].boxplot(red_x, labels=labels)
axes_red[0].set_title('X Errors')
axes_red[0].set_ylabel('Error')

axes_red[1].boxplot(red_y, labels=labels)
axes_red[1].set_title('Y Errors')

axes_red[2].boxplot(red_z, labels=labels)
axes_red[2].set_title('Z Errors')

plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show(block=False)


all_x_errors = [
    errors_dict_case1['blue_x_errors'],
    errors_dict_case2['blue_x_errors'],
    errors_dict_case3['blue_x_errors']
]


# Save Blue Target Box Plot with 300 DPI and bounded pixel dimensions
fig_blue.set_size_inches(10, 5)  # Width × Height in inches
fig_blue.savefig("blue_target_boxplots.png", dpi=300, bbox_inches='tight')

# Save Red Target Box Plot with 300 DPI and bounded pixel dimensions
fig_red.set_size_inches(10, 5)
fig_red.savefig("red_target_boxplots.png", dpi=300, bbox_inches='tight')



# Keep the plots open
input("Press Enter to close the plots...")