import time
import numpy as np
import skfuzzy as fuzz
import matplotlib.pyplot as plt

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class fuzzy_pid_controller:

#This was originally a PD controller, MBK added the Integral term, hence converitng it into a PID
#MBK has tested this code

 
    def __init__(self, p_coef, i_coef ,d_coef, limit_out):
        self._p_coef = p_coef
        self._i_coef = i_coef
        self._d_coef = d_coef
        self._limit_out = limit_out

        self._previous_error = 0.0
        self._integral_of_the_error = 0.0
        self._is_error_initialized = False


        #help taken from
        #https://github.com/mick001/PID-Controller/blob/master/python_code/controllerPID.py
        # Timing        
        self.now_time = time.time()
        self.old_time = self.now_time       

        #help taken from 
        #https://github.com/ivmech/ivPID
        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

    def set_Kp(self,Kp):
        self._p_coef=Kp

    def set_Kd(self,Kd):
        self._d_coef=Kd

    def set_Ki(self,Ki):
        self._i_coef=Ki

    def set_limit_out(self,limit_out):
        self._limit_out=limit_out
    


    def set_current_error(self, error):
        
        # Get elapsed time
        self.now_time = time.time()        
        dt = self.now_time - self.old_time


        if self._is_error_initialized:
            
            
            #Adding the contribution due to 'P' controller
            output_Kp = error * self._p_coef

            #Adding the contribution due to 'D' controller
            if dt > 0:
                error_diff = (error - self._previous_error)/dt
            #Adding the contribution due to 'D' controller
            output_Kd= self._d_coef * error_diff

         
                       
            #Adding the contribution due to 'I' controller
            self._integral_of_the_error +=  error*dt
            

            if (self._integral_of_the_error < -self.windup_guard):
                self._integral_of_the_error = -self.windup_guard
            elif (self._integral_of_the_error > self.windup_guard):
                self._integral_of_the_error = self.windup_guard

            error_integration=self._integral_of_the_error
            # Define the universe of discourse for error (e), derivative of error (de), and integral of error (ie)
            universe = np.linspace(-1, 1, 100)

            # Define the membership labels
            membership_labels = ['NL', 'NS', 'ZERO', 'PS', 'PL']

            # Function to create membership functions for an antecedent or consequent
            def create_membership_functions(variable, universe):
                variable['NL'] = fuzz.trimf(universe, [-1, -1, -0.5])
                variable['NS'] = fuzz.trimf(universe, [-1, -0.5, 0])
                variable['ZERO'] = fuzz.trimf(universe, [-0.5, 0, 0.5])
                variable['PS'] = fuzz.trimf(universe, [0, 0.5, 1])
                variable['PL'] = fuzz.trimf(universe, [0.5, 1, 1])

            # Function to generate all fuzzy rules
            def generate_fuzzy_rules(error, d_error, i_error, output):
                rules = []
                for e in membership_labels:
                    for de in membership_labels:
                        for ie in membership_labels:
                            # Define a rule: if error is 'e', d_error is 'de', and i_error is 'ie'
                            # This example uses a simple approach where output is 'e' (you can modify this logic)
                            rule = ctrl.Rule(error[e] & d_error[de] & i_error[ie], output[e])
                            rules.append(rule)
                return rules

            # Create fuzzy variables for error, derivative of error, integral of error, and output
            error = ctrl.Antecedent(universe, 'error')
            d_error = ctrl.Antecedent(universe, 'd_error')
            i_error = ctrl.Antecedent(universe, 'i_error')
            output = ctrl.Consequent(universe, 'output')

            # Create membership functions for each fuzzy variable
            create_membership_functions(error, universe)
            create_membership_functions(d_error, universe)
            create_membership_functions(i_error, universe)
            create_membership_functions(output, universe)

            # Generate the fuzzy rules
            fuzzy_rules = generate_fuzzy_rules(error, d_error, i_error, output)

            # Build the control system with all 125 rules
            control_system = ctrl.ControlSystem(fuzzy_rules)
            fuzzy_pid = ctrl.ControlSystemSimulation(control_system)

            # Example simulation with some input values for error, derivative of error, and integral of error
            # print('error length')
            # print(len(error))
            # print(error)
            # for i in range(len(error)):
            #     fuzzy_pid.input['error'] = error[i]
            #     fuzzy_pid.input['d_error'] = error_diff[i]
            #     fuzzy_pid.input['i_error'] = error_integration[i]

            #     fuzzy_pid.compute()

            #     print("Computed crisp output u_x:", fuzzy_pid.output['u_x'])
                
            #     #Total control signal
            #     # output=output_Kp+self._i_coef*self._output_Ki +output_Kd  
            #     output=fuzzy_pid.output['u_x']

            #     self._previous_error = error
            fuzzy_pid.input['error'] = error
            fuzzy_pid.input['d_error'] = error_diff
            fuzzy_pid.input['i_error'] = error_integration

            # Compute the fuzzy PID output
            fuzzy_pid.compute()

            print("Computed crisp output u_x:", fuzzy_pid.output['u_x'])
            
            #Total control signal
            # output=output_Kp+self._i_coef*self._output_Ki +output_Kd  
            output=fuzzy_pid.output['u_x']

            self._previous_error = error
        else:
            print("I am here")
            self._previous_error = error
            self._is_error_initialized = True
            self._integral_of_the_error = 0
            output=0
            if output > self._limit_out:
                output = self._limit_out
            elif output < (-self._limit_out):
                output = (-self._limit_out)
        return output


    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time