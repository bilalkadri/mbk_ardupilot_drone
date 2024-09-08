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
        self._output_Ki = 0.0
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
            self._output_Ki +=  error*dt
            

            if (self._output_Ki < -self.windup_guard):
                self._output_Ki = -self.windup_guard
            elif (self._output_Ki > self.windup_guard):
                self._output_Ki = self.windup_guard

            # Define the universe of discourse for inputs and output
            x_range = np.linspace(0, 1, 100)
            u_range = np.linspace(0, 1, 100)

            # Define fuzzy variables
            e_x = ctrl.Antecedent(x_range, 'e_x')
            de_x = ctrl.Antecedent(x_range, 'de_x')
            u_x = ctrl.Consequent(u_range, 'u_x')

            # Define membership functions for e_x and de_x
            e_x['NL'] = fuzz.trimf(e_x.universe, [0, 0, 0.25])
            e_x['NS'] = fuzz.trimf(e_x.universe, [0, 0.25, 0.5])
            e_x['ZE'] = fuzz.trimf(e_x.universe, [0.25, 0.5, 0.75])
            e_x['PS'] = fuzz.trimf(e_x.universe, [0.5, 0.75, 1])
            e_x['PL'] = fuzz.trimf(e_x.universe, [0.75, 1, 1])

            de_x['NL'] = fuzz.trimf(de_x.universe, [0, 0, 0.25])
            de_x['NS'] = fuzz.trimf(de_x.universe, [0, 0.25, 0.5])
            de_x['ZE'] = fuzz.trimf(de_x.universe, [0.25, 0.5, 0.75])
            de_x['PS'] = fuzz.trimf(de_x.universe, [0.5, 0.75, 1])
            de_x['PL'] = fuzz.trimf(de_x.universe, [0.75, 1, 1])

            # Define membership functions for u_x (output)
            u_x['PVS'] = fuzz.trimf(u_x.universe, [0, 0, 0.16])
            u_x['PS'] = fuzz.trimf(u_x.universe, [0, 0.16, 0.33])
            u_x['PMS'] = fuzz.trimf(u_x.universe, [0.16, 0.33, 0.5])
            u_x['PM'] = fuzz.trimf(u_x.universe, [0.33, 0.5, 0.66])
            u_x['PML'] = fuzz.trimf(u_x.universe, [0.5, 0.66, 0.83])
            u_x['PL'] = fuzz.trimf(u_x.universe, [0.66, 0.83, 1])
            u_x['PVL'] = fuzz.trimf(u_x.universe, [0.83, 1, 1])

            # Define fuzzy rules based on the provided table
            rule1 = ctrl.Rule(e_x['NL'] & de_x['NL'], u_x['PVL'])
            rule2 = ctrl.Rule(e_x['NL'] & de_x['NS'], u_x['PVL'])
            rule3 = ctrl.Rule(e_x['NL'] & de_x['ZE'], u_x['PVL'])
            rule4 = ctrl.Rule(e_x['NL'] & de_x['PS'], u_x['PVL'])
            rule5 = ctrl.Rule(e_x['NL'] & de_x['PL'], u_x['PVL'])

            rule6 = ctrl.Rule(e_x['NS'] & de_x['NL'], u_x['PML'])
            rule7 = ctrl.Rule(e_x['NS'] & de_x['NS'], u_x['PML'])
            rule8 = ctrl.Rule(e_x['NS'] & de_x['ZE'], u_x['PML'])
            rule9 = ctrl.Rule(e_x['NS'] & de_x['PS'], u_x['PML'])
            rule10 = ctrl.Rule(e_x['NS'] & de_x['PL'], u_x['PL'])

            rule11 = ctrl.Rule(e_x['ZE'] & de_x['NL'], u_x['PVS'])
            rule12 = ctrl.Rule(e_x['ZE'] & de_x['NS'], u_x['PVS'])
            rule13 = ctrl.Rule(e_x['ZE'] & de_x['ZE'], u_x['PS'])
            rule14 = ctrl.Rule(e_x['ZE'] & de_x['PS'], u_x['PMS'])
            rule15 = ctrl.Rule(e_x['ZE'] & de_x['PL'], u_x['PMS'])

            rule16 = ctrl.Rule(e_x['PS'] & de_x['NL'], u_x['PML'])
            rule17 = ctrl.Rule(e_x['PS'] & de_x['NS'], u_x['PML'])
            rule18 = ctrl.Rule(e_x['PS'] & de_x['ZE'], u_x['PML'])
            rule19 = ctrl.Rule(e_x['PS'] & de_x['PS'], u_x['PL'])
            rule20 = ctrl.Rule(e_x['PS'] & de_x['PL'], u_x['PVL'])

            rule21 = ctrl.Rule(e_x['PL'] & de_x['NL'], u_x['PVL'])
            rule22 = ctrl.Rule(e_x['PL'] & de_x['NS'], u_x['PVL'])
            rule23 = ctrl.Rule(e_x['PL'] & de_x['ZE'], u_x['PMS'])
            rule24 = ctrl.Rule(e_x['PL'] & de_x['PS'], u_x['PL'])
            rule25 = ctrl.Rule(e_x['PL'] & de_x['PL'], u_x['PVL'])

            # Control system and simulation
            control_system = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, 
                                                rule6, rule7, rule8, rule9, rule10, 
                                                rule11, rule12, rule13, rule14, rule15, 
                                                rule16, rule17, rule18, rule19, rule20, 
                                                rule21, rule22, rule23, rule24, rule25])

            fuzzy_sim = ctrl.ControlSystemSimulation(control_system)

            # Test with sample inputs
            fuzzy_sim.input['e_x'] = error
            fuzzy_sim.input['de_x'] = error_diff
           
            # Compute output using centroid defuzzification
            fuzzy_sim.compute()

            print("Computed crisp output u_x:", fuzzy_sim.output['u_x'])
            
            #Total control signal
            # output=output_Kp+self._i_coef*self._output_Ki +output_Kd  
            output=fuzzy_sim.output['u_x']

            self._previous_error = error
        else:
            print("I am here")
            self._previous_error = error
            self._is_error_initialized = True
            self._output_Ki = 0
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