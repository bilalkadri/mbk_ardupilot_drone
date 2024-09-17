import time
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

class fuzzy_pid_controller:
    def __init__(self, p_coef, i_coef, d_coef, limit_out):
        self._p_coef = p_coef
        self._i_coef = i_coef
        self._d_coef = d_coef
        self._limit_out = limit_out
        self._previous_error = 0.0
        self._integral_of_the_error = 0.0
        self._is_error_initialized = False
        self.now_time = time.time()
        self.old_time = self.now_time
        self.int_error = 0.0
        self.windup_guard = 20.0

        # Precompute fuzzy membership functions and rules
        self.universe = np.linspace(-1, 1, 50)  # Reduced resolution for performance
        self.membership_labels = ['NL', 'NS', 'ZERO', 'PS', 'PL']

        self.err_antecedent = ctrl.Antecedent(self.universe, 'error')
        self.d_err_antecedent = ctrl.Antecedent(self.universe, 'd_error')
        self.i_err_antecedent = ctrl.Antecedent(self.universe, 'i_error')
        self.output_consequent = ctrl.Consequent(self.universe, 'output')

        self.create_membership_functions(self.err_antecedent)
        self.create_membership_functions(self.d_err_antecedent)
        self.create_membership_functions(self.i_err_antecedent)
        self.create_membership_functions(self.output_consequent)

        self.fuzzy_rules = self.generate_fuzzy_rules(self.err_antecedent, self.d_err_antecedent, self.i_err_antecedent, self.output_consequent)

        # Build the fuzzy control system once
        self.control_system = ctrl.ControlSystem(self.fuzzy_rules)
        self.fuzzy_pid = ctrl.ControlSystemSimulation(self.control_system)

    def create_membership_functions(self, variable):
        variable['NL'] = fuzz.trimf(self.universe, [-1, -1, -0.5])
        variable['NS'] = fuzz.trimf(self.universe, [-1, -0.5, 0])
        variable['ZERO'] = fuzz.trimf(self.universe, [-0.5, 0, 0.5])
        variable['PS'] = fuzz.trimf(self.universe, [0, 0.5, 1])
        variable['PL'] = fuzz.trimf(self.universe, [0.5, 1, 1])

    def generate_fuzzy_rules(self, err, derr, ierr, out):
        rules = []
        for e in self.membership_labels:
            for de in self.membership_labels:
                for ie in self.membership_labels:
                    rule = ctrl.Rule(err[e] & derr[de] & ierr[ie], out[e])
                    rules.append(rule)
        return rules

    def set_current_error(self, error_value):
        # Get elapsed time
        self.now_time = time.time()        
        dt = self.now_time - self.old_time

        if self._is_error_initialized:
            # P Controller
            output_Kp = error_value * self._p_coef

            # D Controller
            if dt > 0:
                error_diff = (error_value - self._previous_error) / dt
            output_Kd = self._d_coef * error_diff

            # I Controller
            self._integral_of_the_error += error_value * dt

            if self._integral_of_the_error < -self.windup_guard:
                self._integral_of_the_error = -self.windup_guard
            elif self._integral_of_the_error > self.windup_guard:
                self._integral_of_the_error = self.windup_guard

            error_integration = self._integral_of_the_error

            # Set fuzzy inputs
            self.fuzzy_pid.input['error'] = error_value
            self.fuzzy_pid.input['d_error'] = error_diff
            self.fuzzy_pid.input['i_error'] = error_integration

            # Compute fuzzy output
            self.fuzzy_pid.compute()

            output_fuzzy = self.fuzzy_pid.output['output']
            # print("Computed crisp output u_x:", output_fuzzy)

            output = output_fuzzy
            self._previous_error = error_value

        else:
            self._previous_error = error_value
            self._is_error_initialized = True
            self._integral_of_the_error = 0
            output = 0
        
        if output > self._limit_out:
            output = self._limit_out
        elif output < -self._limit_out:
            output = -self._limit_out

        self.old_time = self.now_time  # Update old time for next iteration
        return output

    def setWindup(self, windup):
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time
