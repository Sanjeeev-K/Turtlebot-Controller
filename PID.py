#!usr/bin/env python
#Code to publish velocity and store odom data in a variable.

#import packages used in the program
import rospy
import math
from matplotlib import pyplot as plt
import time

from nav_msgs.msg import Odometry #to get input from /odom for localization
from geometry_msgs.msg import Twist #to publish velocity messages to the robot

def callback(data):
    # rospy.loginfo("x = %s y = %s", data.pose.pose.position.x, data.pose.pose.position.y)
    global x_curr, y_curr, x_prev, y_prev, q_z, q_w 
    x_prev = x_curr #capture the previous location
    y_prev = y_curr
    x_curr = data.pose.pose.position.x #actuate
    y_curr = data.pose.pose.position.y
    q_z    = data.pose.pose.orientation.z
    q_w    = data.pose.pose.orientation.w

# global x_curr, y_curr
x_curr = 0  #initialize values
y_curr = 0
x_prev = -1
y_prev = -1
q_z = 0
q_w = 0

def myhook(): #function to shutdown tbot
  rospy.loginfo("shutdown time!")

def publish_plus_subscribe():
    print("Hello")
    pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10) 
    #<name of publisher> = rospy.Publisher(<topic name, within '' or "", msg type,buffer size)
    rospy.init_node('node_name', anonymous=True)
    rospy.Subscriber('/odom',Odometry,callback)
    #rospy.init_node(<Name of Node > )
    # This name is extremely important, as this name will be used by ROS to refer to this script.
    #Required only once per script.
    #anonymous=True is required for ensuring node name is unique.
    freq = 100 #rate at which simulation is run
    rate = rospy.Rate(freq) 
    x_des = 3 #desired coordinates
    y_des = 3
    tolerance = 0.1 #value to assess destination is reached
    kp_lin = 0.08 #linear positional
    kp_ang = 0.6 #angular positional 
    kd_lin = 0.1 #linear differential
    kd_ang = 0.1 #angular differential
    ki_lin = 0.01 #linear integral
    ki_ang = 0.001 #angular integral
    error_lin_prev = math.sqrt( (x_des - x_curr)**2 + (y_des - y_curr)**2  )
    error_ang_prev = math.atan2( (y_des - y_curr) , (x_des - x_curr) ) - math.atan2(y_curr - y_prev,x_curr- x_prev)
    sum_err_lin = 0
    sum_error_ang = 0
    random123 = Twist()
    pub.publish(random123)
    a123 = rospy.Time.now()
    b = a123.secs
	
	#initialize lists for graphs
    time_list = []
    c = []
    error_lin_list = []
    error_ang_list = []
    x_list=[]
    y_list =[]
    while not rospy.is_shutdown(): #run this loop until shutdown (<tolerance)
		# calculate errors (position and velocity)
        error_lin = math.sqrt( (x_des - x_curr)**2 + (y_des - y_curr)**2  )
        d_error_lin = (error_lin - error_lin_prev) * freq
        error_lin_prev = error_lin
        sum_err_lin = sum_err_lin + error_lin
        vel = kp_lin * error_lin + kd_lin * d_error_lin + ki_lin* sum_err_lin
        theta1 = math.atan2(  2*(q_w*q_z) , (1 - 2*q_z*q_z  )  )

        error_ang = math.atan2( (y_des - y_curr) , (x_des - x_curr) ) - theta1
        
        d_error_ang = (error_ang - error_ang_prev) * freq
        error_ang_prev = error_ang
        sum_error_ang = sum_error_ang + error_ang
        turn = kp_ang * error_ang + kd_ang * d_error_ang + ki_ang*sum_error_ang

        a = Twist()
        a.angular.z = turn

        if vel>0: #linear velocity limiter
            a.linear.x = min(vel,0.3)
        else:
            a.linear.x = max(vel,-0.3)
        
        rospy.loginfo(error_ang)

        d = rospy.Time.now()
        time_list.append(d.secs)
        rospy.loginfo(d.secs)
        if (len(time_list)<2):
            rospy.loginfo(time_list[0])
            c.append(time_list[0])
        else:
            rospy.loginfo(d.secs - time_list[1])
            c.append(d.secs - time_list[1])

		#add data to lists for graphs
        error_lin_list.append(error_lin)
        error_ang_list.append(error_ang)
        x_list.append(x_curr)
        y_list.append(y_curr)


        pub.publish(a)

		#if the destination is reached, call shutdown and plot data
        if error_lin<tolerance:
            rospy.loginfo("Reached Destination")
            rospy.signal_shutdown(myhook)
            plt.figure(1)
            plt.plot(c,error_lin_list)
            plt.ylim(0,50) 
            plt.show()
            plt.figure(2)
            plt.plot(c,error_ang_list)

            plt.show()

            plt.figure(3)
            plt.plot(x_list,y_list)
            plt.show()

        rate.sleep()

#call to initialization function
if __name__ == '__main__':
    try:
        publish_plus_subscribe()
    except rospy.ROSInterruptException:
        rospy.loginfo("Terminating Program...")
        pass








