import time
import numpy as np
import skfuzzy as fuzz
import matplotlib.pyplot as plt
from skfuzzy import control as ctrl

class FuzzyPIDController:

    def __init__(self, p_coef, i_coef, d_coef, limit_out):
        self._p_coef = p_coef
        self._i_coef = i_coef
        self._d_coef = d_coef
        self._limit_out = limit_out

        self._previous_error = 0.0
        self._integral_of_the_error = 0.0
        self._is_error_initialized = False

        # Timing        
        self.now_time = time.time()
        self.old_time = self.now_time       

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

    def set_Kp(self, Kp):
        self._p_coef = Kp

    def set_Kd(self, Kd):
        self._d_coef = Kd

    def set_Ki(self, Ki):
        self._i_coef = Ki

    def set_limit_out(self, limit_out):
        self._limit_out = limit_out

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

            # Define the universe of discourse for error, d_error, and i_error
            universe = np.linspace(-1, 1, 100)
            membership_labels = ['NL', 'NS', 'ZERO', 'PS', 'PL']

            def create_membership_functions(variable, universe):
                variable['NL'] = fuzz.trimf(universe, [-1, -1, -0.5])
                variable['NS'] = fuzz.trimf(universe, [-1, -0.5, 0])
                variable['ZERO'] = fuzz.trimf(universe, [-0.5, 0, 0.5])
                variable['PS'] = fuzz.trimf(universe, [0, 0.5, 1])
                variable['PL'] = fuzz.trimf(universe, [0.5, 1, 1])

            def generate_fuzzy_rules(err, derr, ierr, out):
                rules = []
                for e in membership_labels:
                    for de in membership_labels:
                        for ie in membership_labels:
                            rule = ctrl.Rule(err[e] & derr[de] & ierr[ie], out[e])
                            rules.append(rule)
                return rules

            # Create fuzzy variables and rename to avoid conflicts
            err_antecedent = ctrl.Antecedent(universe, 'error')
            d_err_antecedent = ctrl.Antecedent(universe, 'd_error')
            i_err_antecedent = ctrl.Antecedent(universe, 'i_error')
            output_consequent = ctrl.Consequent(universe, 'output')

            create_membership_functions(err_antecedent, universe)
            create_membership_functions(d_err_antecedent, universe)
            create_membership_functions(i_err_antecedent, universe)
            create_membership_functions(output_consequent, universe)

            fuzzy_rules = generate_fuzzy_rules(err_antecedent, d_err_antecedent, i_err_antecedent, output_consequent)

            # Build and simulate fuzzy control system
            control_system = ctrl.ControlSystem(fuzzy_rules)
            fuzzy_pid = ctrl.ControlSystemSimulation(control_system)

            # Set fuzzy inputs
            fuzzy_pid.input['error'] = error_value
            fuzzy_pid.input['d_error'] = error_diff
            fuzzy_pid.input['i_error'] = error_integration

            # Compute fuzzy output
            fuzzy_pid.compute()

            output_fuzzy = fuzzy_pid.output['output']
            print("Computed crisp output u_x:", output_fuzzy)

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

        return output

    def setWindup(self, windup):
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time
