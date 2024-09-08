from fuzzy_pid_controller_mbk import fuzzy_pid_controller



controller_x = fuzzy_pid_controller(0.01, .01, 2, 1)
# print("I am here")
controller_x.set_current_error(0.1)
