#! /usr/bin/python
#!/usr/bin/env python3
#!/usr/bin/env python2


from scipy.optimize import minimize
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, TwistStamped
import time

#Modifying the code for ARDUPILOT
dronetype='/mavros'

X_Error_list = []
Y_Error_list = []
Control_Signal_X_List=[]
Control_Signal_Y_List=[]
Control_Signal=TwistStamped()

We = 1
Wu = 0.00002
length=10

# stp = np.zeros((length, len(We) + 1))
# command = np.zeros((length, len(We) + 1))
# v = np.ones((length, len(We) + 1))*0.5

def Cost_PID_x(input_args,We,Wu):
    global X_Error_list 
    global Control_Signal_X_List

    #I must get the [Kp, Ki, Kd] values chosen by the minimze function
    # and set these values on the actual PID Controller. I can do this 
    #by sending these values to ROSPARAM server. The quad must use these 
    #values and generate some errors these error values can be used for computing the 
    #new cost 
    Kp=float(input_args[0])
    Ki=float(input_args[1])
    Kd=float(input_args[2])
    rospy.set_param('/Kp_x',Kp)
    rospy.set_param('/Ki_x',Ki)
    rospy.set_param('/Kd_x',Kd)
    # limit=rospy.set_param('limit')
    
    # I can also set the parameters on the ROS-Server using the following syntax 
    # rospy.set_param('gains', {'p': 1, 'i': 2, 'd': 3})
    #---------------------------
    # fetch a group (dictionary) of parameters
    # gains = rospy.get_param('gains')
    # p, i, d = gains['p'], gains['i'], gains['d']

    #I must sleep here for a while for the errors to accumulate, these 
    # errors will correspond to the current Kp, Ki, Kd Vaules
    time.sleep(1)

    #Returning the last 20 elements from the list
    X_Error_list_for_computation=X_Error_list[-20:]
    Control_Signal_X_List_for_computation=Control_Signal_X_List[-20:]

    if len(X_Error_list_for_computation)==0:
        X_Error_list_for_computation=[0.01]*20
    # if len(Control_Signal_X_List_for_computation)==0:
    #     Control_Signal_X_List_for_computation=[0.01]*20
    
    # Check if the list is empty
    if not Control_Signal_X_List_for_computation:
    # If the list is empty, append 0.1 to it
        Control_Signal_X_List_for_computation.append(0.1)
   
    
    cost = (
        np.sum(np.square(X_Error_list_for_computation)) * We
        + np.sum(np.square(np.diff(Control_Signal_X_List_for_computation))) * Wu
        + Control_Signal_X_List_for_computation[0] * Control_Signal_X_List_for_computation[0] * Wu
        )
    # J = sum((stp[i] - v[i])^2*t[i])*We + sum((command[i+1] - command[i])^2)*Wu + command[0]^2*Wu
    # cost = np.sum(np.square(X_Error_list_for_computation)) * We + np.sum(np.square(np.diff(Control_Signal_X_List_for_computation))) * Wu + Control_Signal_X_List_for_computation[0] * Control_Signal_X_List_for_computation[0] * Wu

    return cost 

def Cost_PID_y(input_args,We,Wu):
    global Y_Error_list 
    global Control_Signal_Y_List

    #I must get the [Kp, Ki, Kd] values chosen by the minimze function
    # and set these values on the actual PID Controller. I can do this 
    #by sending these values to ROSPARAM server. The quad must use these 
    #values and generate some errors these error values can be used for computing the 
    #new cost 
    Kp=float(input_args[0])
    Ki=float(input_args[1])
    Kd=float(input_args[2])
    rospy.set_param('/Kp_y',Kp)
    rospy.set_param('/Ki_y',Ki)
    rospy.set_param('/Kd_y',Kd)
          
    # limit=rospy.set_param('limit')
    #I must sleep here for a while for the errors to accumulate, these 
    # errors will correspond to the current Kp, Ki, Kd Vaules
    time.sleep(1)


    #Returning the last 20 elements from the list
    Y_Error_list_for_computation=Y_Error_list[-20:]
    Control_Signal_Y_List_for_computation=Control_Signal_Y_List[-20:]
    
    if len(Y_Error_list_for_computation)==0:
        Y_Error_list_for_computation=[0.01]*20
    # if len(Control_Signal_Y_List_for_computation)==0:
    #     Control_Signal_Y_List_for_computation=[0.01]*20

    # Check if the list is empty
    if not Control_Signal_Y_List_for_computation:
    # If the list is empty, append 0.1 to it
        Control_Signal_Y_List_for_computation.append(0.1)
    
    cost = (
        np.sum(np.square(Y_Error_list_for_computation)) * We
        + np.sum(np.square(np.diff(Control_Signal_Y_List_for_computation))) * Wu
        + Control_Signal_Y_List_for_computation[0] * Control_Signal_Y_List_for_computation[0] * Wu
        )


    # J = sum((stp[i] - v[i])^2*t[i])*We + sum((command[i+1] - command[i])^2)*Wu + command[0]^2*Wu
    cost = np.sum(np.square(Y_Error_list_for_computation)) * We + np.sum(np.square(np.diff(Control_Signal_Y_List_for_computation))) * Wu + Control_Signal_Y_List_for_computation[0] * Control_Signal_Y_List_for_computation[0] * Wu

    return cost 


def Control_Signal_Callback_function(data_recieve):
    global Control_Signal_X_List
    global Control_Signal_Y_List
    global Control_Signal
    
    Control_Signal=data_recieve

    Control_Signal_X=Control_Signal.twist.linear.x
    Control_Signal_Y=Control_Signal.twist.linear.y
    Control_Signal_X_List.append(Control_Signal_X)
    Control_Signal_Y_List.append(Control_Signal_Y)
    
    

def Water_Reservoir_Detected_Callback_function(data_recieve):
    
    global counter_for_BLUE
    global X_Error_list
    global Y_Error_list 
 
    #print('Water Reservoir detected')
    #print data_recieve.data[0]
    setPointX=data_recieve.data[0]  # 320, 640/2 , half the size of x-dimension of window 
    setPointY=data_recieve.data[1]  # 180, 360/2 , half the szie of y-dimension of window
    currentX=data_recieve.data[2]
    currentY=data_recieve.data[3]
    currentDepth=data_recieve.data[4]  #This the radius of the target as calualted by OPenCV library, by adjusting the 
                                       #radius we can control the height of the quadcopter

    X_Error= setPointX-currentX       #Remember these errors are caluclated in the image frame of reference where
    Y_Error= setPointY-currentY       #(x,y) is the top left corner
    X_Error_list.append(X_Error)
    Y_Error_list.append(Y_Error)

   

def Water_Discharge_Detected_Callback_function(data_recieve):
    
    global counter_for_RED
    global X_Error_list
    global Y_Error_list 
 
    #print('Water Reservoir detected')
    #print data_recieve.data[0]
    setPointX=data_recieve.data[0]  # 320, 640/2 , half the size of x-dimension of window 
    setPointY=data_recieve.data[1]  # 180, 360/2 , half the szie of y-dimension of window
    currentX=data_recieve.data[2]
    currentY=data_recieve.data[3]
    currentDepth=data_recieve.data[4]  #This the radius of the target as calualted by OPenCV library, by adjusting the 
                                       #radius we can control the height of the quadcopter

    X_Error= setPointX-currentX       #Remember these errors are caluclated in the image frame of reference where
    Y_Error= setPointY-currentY       #(x,y) is the top left corner
    X_Error_list.append(X_Error)
    Y_Error_list.append(Y_Error)

def main():
    # -------- Configuration --------

    # Optimization weights for cost function
    We = 1
    Wu = 0.00002
    

    # Initialize arrays for storing results

    # t = np.zeros((length, len(We) + 1))
    # stp = np.zeros((length, len(We) + 1))
    # command = np.zeros((length, len(We) + 1))
    # theta = np.zeros((length, len(We) + 1))
    # v = np.zeros((length, len(We) + 1))
    # result = []

    # Perform minimization for each couple of We and Wu weights

    # for idx in range(0, len(We)):
    #     bounds = ((0, None), (0, None), (0, None))
    #     r = minimize(Cost, [500, 3, 3],
    #                  args=(time_step, end_time, We[idx], Wu[idx], uphill[idx]),
    #                  bounds=bounds)
    #     result.append(r)

        

    #     # Print optimization results

    #     print("We = " + "{:.3g}".format(We[idx]) + " Wu = " + "{:.3g}".format(Wu[idx]) + " Uphill = " + "{:.0g}".format(
    #         uphill[idx]) + " Kp = " + "{:.3g}".format(result[idx].x[0])
    #           + " Ki = " + "{:.3g}".format(result[idx].x[1]) + " Kaw = " + "{:.3g}".format(result[idx].x[2]))
    #     print("Success: " + str(r.success))

    #     # # Run simulation with optimized parameters
    #     # (t[:, idx], stp[:, idx], v[:, idx], command[:, idx], theta[:, idx]) = Simulation(r.x, time_step, end_time, m, b,
    #     #                                                                                  F_max_0, F_max_max, v_max, 1)
    
    #Initializing a ROS Node
    rospy.init_node('Gain_Schduled_PID_Controller')
    
    #Fetch the initial values of the Gains from the ROS Parameter Server
    Kp_x=rospy.get_param('/Kp_x')
    Ki_x=rospy.get_param('/Ki_x')
    Kd_x=rospy.get_param('/Kd_x')
    limit_x=rospy.get_param('/limit_x')
    
    bounds = ((0, None), (0, None), (0, None))
    # r_PID_x=minimize(Cost_PID_x,[500,3,3], args=(We, Wu),bounds=bounds)
    # The minimize function will itself call Cost_PID_x multiple times 
    # by varying the values of Kp, Ki and Kd. IT will iteratively reach 
    # towards the solution. For each pair of values it should get the cost
    # by the Cost_PID_x.  
    parameters=[Kp_x,Ki_x,Kd_x]
    r_PID_x=minimize(Cost_PID_x,parameters, args=(We, Wu),bounds=bounds)
    #The gains returned by the minimize function must be saved on the ROS Parameter Server
    print(r_PID_x.x)

    Kp_x=float(r_PID_x.x[0])
    Ki_x=float(r_PID_x.x[1])
    Kd_x=float(r_PID_x.x[2])
    rospy.set_param('/Kp_x',Kp_x)
    rospy.set_param('/Ki_x',Ki_x)
    rospy.set_param('/Kd_x',Kd_x)
    

    Kp_y=rospy.get_param('/Kp_y')
    Ki_y=rospy.get_param('/Ki_y')
    Kd_y=rospy.get_param('/Kd_y')
    limit_y=rospy.get_param('/limit_y')
    
    parameters=[Kp_y,Ki_y,Kd_y]
    r_PID_y=minimize(Cost_PID_y,parameters, args=(We, Wu),bounds=bounds)
    #The gains returned by the minimize function must be saved on the ROS Parameter Server
    print(r_PID_y.x)
    Kp_y=float(r_PID_y.x[0])
    Ki_y=float(r_PID_y.x[1])
    Kd_y=float(r_PID_y.x[2])
    rospy.set_param('/Kp_y',Kp_y)
    rospy.set_param('/Ki_y',Ki_y)
    rospy.set_param('/Kd_y',Kd_y)
    
        
    #I have to define a SUbscriber so that I can compute the error in x and y direction
    Subscriber_1=rospy.Subscriber("/Water_Reservoir_Location_Topic",Float32MultiArray,Water_Reservoir_Detected_Callback_function,queue_size=10)

    Subscriber_2=rospy.Subscriber("/Water_Discharge_Location_Topic",Float32MultiArray,Water_Discharge_Detected_Callback_function,queue_size=10)

    Subscriber_3=rospy.Subscriber(dronetype+'/setpoint_velocity/cmd_vel', TwistStamped,Control_Signal_Callback_function,queue_size=10)

    rospy.spin()
    



main()