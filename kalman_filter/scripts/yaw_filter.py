#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

# Set Topic name 
topicName = "YawAngle"
# Set Flag for initilizaton process
flag = False
# Set inital predicted values for x : angle , p : uncertainity estimate
x_init = 50.0
p_init = 10000

#___________________________________________________________________________________________#

# x_n_n represents Current State Estimate (x_n,n) 
x_n_n = 0
# x_n_n_1 represents Previous State Estimate (x_n,n-1) 
x_n_n_1 = 0
# z_n represents measured value
z_n = 0
# K_n represents Kalman Gain
k_n = 0

# P_n_n represents Current State Estimate Uncertainity [variance] (P_n,n) 
P_n_n = 0
# P_n_n represents previous State Estimate Uncertainity [variance] (P_n,n-1)
P_n_n_1 = 0

#*** These r and q value can be tuned by the user ***#
# r_n represents Measurement Uncertainity [variance]  
r_n = 0.03
# q represents Process Noise : Uncertainity of dynamic model [variance]
q = 0.19
#________________________________Kalman Filter_____________________________________________#

def callback(msg: Float32):
# Taking measurement 
    z_n = msg.data
# Initiliaztion phase
    if(flag == False):
    # Assume Constant Dynamic Model
    # Prediction:        
        x_n_n_1 = x_init
        P_n_n_1 = p_init + q    
        flag = True
#**************************** State Update *****************************************#        

#  Kalman Gain Equation
    k_n = (P_n_n_1) / (P_n_n_1 + r_n)
# State update Equation 
    x_n_n = (x_n_n_1) + (k_n * (z_n - x_n_n_1) )
# Covariance update Equation (Estimate Uncertainity update)
    P_n_n = (1-k_n) * (P_n_n_1)

#********************* State Extrapolation (Prediction) **********************************#    
# Assume Constant Dynamic Model
    x_n_n_1 = x_n_n
# Add process noise due to uncertainity in dynamic model
    P_n_n_1 = P_n_n + q     

#___________________________________________________________________________________________#

# Function for the subcriber to get measured values
def listen():
    # Initilaize the node
    rospy.init_node("kalman_filter", anonymous= True)
    # Create a subsciber instance to get measured values from topic 
    rospy.Subscriber(topicName, Float32, callback)
    # Spin to check for callback while keeping the node running
    rospy.spin()

# Function for the publisher to publish filtered values
def publish():
    # Create a publisher instance to publish on topic "filtered_yaw"
    pub = rospy.Publisher("filtered_yaw", Float32, queue_size = 10)
    # Publish current filtered value
    pub.publish(x_n_n)
    # Spin to check for callback while keeping the node running
    rospy.spin()

#____________________________________________________________________________________________#

if __name__ == '__main__':
    try:
       # Calls the subscriber to get measured values
        listen()
        # Calls the publisher to publish filtered values 
        publish()
    except rospy.ROSInterruptException:
       pass
    