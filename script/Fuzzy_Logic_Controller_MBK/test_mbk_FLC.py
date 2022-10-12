#!/usr/bin/python


#
# This file is part of IvPID.
# Copyright (C) 2015 Ivmech Mechatronics Ltd. <bilgi@ivmech.com>
#
# IvPID is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# IvPID is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

#title           :test_pid.py
#description     :python pid controller test
#author          :Muhammad Bilal Kadri
#date            :2021-08-08
#version         :0.1
#notes           :
#python_version  :2.7
#dependencies    : matplotlib, numpy, scipy
#==============================================================================


from FLC_mbk import fuzzy_logic_controller
import time
import matplotlib.pyplot as plt
import numpy as np
# from scipy.interpolate import spline
# from scipy.interpolate import BSpline, make_interp_spline #  Switched to BSpline




def test_FLC(P = 0.2,  I = 0.0, D= 0.0, L=100):
   
    # pid = PID.PID(P, I, D)
    # pid.SetPoint=0.0
    # pid.setSampleTime(0.01)

    MFFAC=fuzzy_logic_controller(P,I,D,20,5)     
    
    #Testing Fuzzification function
    #value=pid.Fuzzification(10.125)
    #print(value)

    #Testing FLMS function
    #print('The old weights are:',pid._weights)
    #pid.FLMS_55([0.1, 0.2])
    #print('The new weights are:',pid._weights)
    
    
    Control_Signal=MFFAC.Executing_FLC([0.1,0.2])
    print('Control Signal={0}'.format(Control_Signal))
    
    
    # END = L
    # feedback = 0
    # SetPoint=0
    # feedback_list = []
    # time_list = []
    # setpoint_list = []

    # for i in range(1, END):
        
    #     error=SetPoint-feedback
    #     output = pid.set_current_error(error)
    #     if SetPoint > 0:
    #         feedback += (output - (1/i))
    #     if i>9:
    #         SetPoint = 1
    #     time.sleep(0.02)

    #     feedback_list.append(feedback)
    #     setpoint_list.append(SetPoint)
    #     time_list.append(i)

    # time_sm = np.array(time_list)
    # time_smooth = np.linspace(time_sm.min(), time_sm.max(), 300)

    # # feedback_smooth = spline(time_list, feedback_list, time_smooth)
    # # Using make_interp_spline to create BSpline
    # helper_x3 = make_interp_spline(time_list, feedback_list)
    # feedback_smooth = helper_x3(time_smooth)

    # plt.plot(time_smooth, feedback_smooth)
    # plt.plot(time_list, setpoint_list)
    # plt.xlim((0, L))
    # plt.ylim((min(feedback_list)-0.5, max(feedback_list)+0.5))
    # plt.xlabel('time (s)')
    # plt.ylabel('PID (PV)')
    # plt.title('TEST PID')

    # plt.ylim((1-0.5, 1+0.5))

    # plt.grid(True)
    # plt.show()

if __name__ == "__main__":
    
    
    #Calling Mamdani FLC from Simpful library (developed in Python3)
    test_FLC(1.2, 1, 0.001, L=100)
#    test_pid(0.8, L=50)
