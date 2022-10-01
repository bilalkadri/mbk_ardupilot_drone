import time


class fuzzy_logic_controller:

#This was originally a PD controller, MBK added the Integral term, hence converitng it into a PID
#MBK has to test this code

 
    def __init__(self, p_coef, i_coef ,d_coef, limit_out,number_of_MF):
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

        #Variables for FLC
        self.number_of_membership_functions=number_of_MF
        
        self._PossIr=[0]*number_of_MF
        self._PossIy=[0]*number_of_MF
        self._delta=0.1
        self._gamma=0.9
        self._number_of_rules=number_of_MF*number_of_MF
        self._F_Array=[10]*(self._number_of_rules)
        self._new_weights=[0.01]*(self._number_of_rules)
        self._old_weights=[0.01]*(self._number_of_rules)
        self._rule_strength=[0]*self._number_of_rules
        self._uf_e_new=0.01
        self._uf_e_old=0.01

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

    def Fuzzification(self, r):

        #defining local variables
        PossIx=[0]*self.number_of_membership_functions
        if r<0:
            r=0
        elif r>1:
            r=1
        elif r>=0  or r<=1:
            r=r

        count=1
        rc=[0]*15

        rc[0]=-0.25
        rc[1]=0
        rc[2]=0.25

        rc[3]=0
        rc[4]=0.25
        rc[5]=0.50

        rc[6]=0.25
        rc[7]=0.50
        rc[8]=0.75

        rc[9]=0.50
        rc[10]=0.75
        rc[11]=1.0

        rc[12]=0.75
        rc[13]=1.0
        rc[14]=1.25


        i=0
        for i in range(self.number_of_membership_functions):
            var1=self.function_U(r,rc[count-1],rc[count])
            var2=self.function_U(r,rc[count],rc[count+1])
            PossIx[i]=(r-rc[count-1])/(rc[count]-rc[count-1])*var1+(rc[count+1]-r)/(rc[count+1]-rc[count])*var2
            count=count+3
        

        if r<0:
            PossIx[0]=1
            PossIx[1:4]=0
        elif r>1:
            PossIx[0:3]=0
            PossIx[4]=1
    
        return PossIx 

    def function_U(self,r,c1,c2):
              
        if r>=c1 and r<c2:
            x=1
        else:
            x=0
        return x
            



    def FLMS_55(self,e):

        #Function arguments
        y=e[0]     # Plant output
        r=e[1]     # Reference Signal
        #uf_e=e[2]  # Expected true value computed from Feedback Error Learning (FEL) Module
        
        #Defining error
        error=r-y

        #Local Variables definitions and initializations
        uf_hat=0        
        
        
        num=[0]*self._number_of_rules

        self._PossIy=self.Fuzzification(y)
        self._PossIr=self.Fuzzification(r) #(Already computed in Fuzzufication)

        count=0
        g=0
        h=0
        for g in range(self.number_of_membership_functions):
            for h in range(self.number_of_membership_functions):
                self._rule_strength[count]=self._PossIy[g]*self._PossIr[h]
                count=count+1
    
        
        # %----------------------------------------------
        # %-----------------F-ARRAY----------------------
        # %----------------------------------------------

        i=0
        for i in range(self._number_of_rules):
            if self._rule_strength[i]>0:
                self._F_Array[i]=self._F_Array[i]+self._rule_strength[i]
            else:
                self._F_Array[i]=self._F_Array[i]
        
            
        # %----------------------------------------------
        # %-------------DENOMINATOR----------------------
        # %----------------------------------------------

        b=1
        den=0
        i=0
        j=0
        for i in range(self._number_of_rules):
            for j in range(self._number_of_rules):
                if j==i:
                    b=b*self._rule_strength[i]*self._rule_strength[i]
                else:
                    b=b*self._F_Array[j]
                
            den=den+b
            b=1
        

        if den==0:
            den=0.01
        
        # %----------------------------------------------
        # %---------------NUMERATOR----------------------
        # %----------------------------------------------

        b=1
           

        for i in range(self._number_of_rules):
            for j in range(self._number_of_rules):
                if j==i:
                    b=b*self._rule_strength[i]
                else:
                    b=b*self._F_Array[j]
                
            
            num[i]=b
            b=1
        

        #%----------------------------------------------
        i=0
        for i in range(self._number_of_rules):
            uf_hat=uf_hat+self._rule_strength[i]*self._old_weights[i]
              
        #The uf_hat must be computed with the old weights
        #The uf_e_new and uf_e_old signal are coming from feedback error learning block    
        self._uf_e_new=self._uf_e_old+self._gamma*error

        #Computing epsilon for weight update equation
        epsilon=(self._uf_e_new-uf_hat)

        #Saving the uf_e_new in uf_e_old for the next iteration
        self._uf_e_old=self._uf_e_new


        i=0
        for i in range(self._number_of_rules):
            self._new_weights[i]=self._old_weights[i]+(self._delta*num[i]*epsilon)/den
        


    def FFC_FLMS_55(self):

        
        i=0
        for i in range(self._number_of_rules):
            self._uf_e_new=self._uf_e_new+self._rule_strength[i]*self._new_weights[i]
        

        #Applying saturation on the control signal
        if self._uf_e_new>1:
            self._uf_e_new=1
        elif self._uf_e_new<0:
            self._uf_e_new=0

        return self._uf_e_new

    def Executing_FLC(self,e):
        r= e[0] #reference signal
        y= e[1] # plant output

        self.FLMS_55([r,y])
        control_signal_uf=self.FFC_FLMS_55()
        return control_signal_uf


