import time
import control.matlab as mtl
class pid_controller:

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

            
            
            #Total control signal
            output=output_Kp+self._i_coef*self._output_Ki +output_Kd   
            self._previous_error = error
        else:
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
    
    def transferFunction(self,N=15):
        
        """ PID transfer function """        
        
        r = self.kd + self.ki * mtl.tf([1],[1,0]) + self.kd * mtl.tf([1,0],[self.kd/self.kp*N,1])        
        return r