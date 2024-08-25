import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, TwistStamped

Control_Signal=TwistStamped()

#Modifying the code for ARDUPILOT
dronetype='/mavros'


def Control_Signal_Callback_function(data_recieve):
    global Control_Signal_X_List
    global Control_Signal_Y_List
    global Control_Signal
    
    Control_Signal=data_recieve

    Control_Signal_X=Control_Signal.twist.linear.x
    Control_Signal_Y=Control_Signal.twist.linear.y

    print("Control signal for PID(x)=",Control_Signal_X)
    print("Control signal for PID(y)=",Control_Signal_Y)
    
    

def main():
        
    #Initializing a ROS Node
    rospy.init_node('Gain_Schduled_PID_Controller')
    Subscriber_3=rospy.Subscriber(dronetype+'/setpoint_velocity/cmd_vel', TwistStamped,Control_Signal_Callback_function,queue_size=10)

    rospy.spin()


main()