#!usr/bin/env python
#Code to publish velocity and store odom data in a variable.
import rospy
import math
from matplotlib import pyplot as plt
import time

from nav_msgs.msg import Odometry #to get input from /odom for localization
from geometry_msgs.msg import Twist #to publish velocity messages to the robot

def callback(data):
    # rospy.loginfo("x = %s y = %s", data.pose.pose.position.x, data.pose.pose.position.y)
    global x_curr, y_curr, x_prev, y_prev, q_z, q_w
    x_prev = x_curr
    y_prev = y_curr
    x_curr = data.pose.pose.position.x
    y_curr = data.pose.pose.position.y
    q_z    = data.pose.pose.orientation.z
    q_w    = data.pose.pose.orientation.w

# global x_curr, y_curr
x_curr = 0  #initial values. If an initial value isn't provided, an error is returned.
y_curr = 0
x_prev = -1
y_prev = -1
q_z = 0
q_w = 0

def myhook():
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
    freq = 100
    rate = rospy.Rate(freq) 
    x_des = 3
    y_des = 3
    tolerance = 0.1
    kp_lin = 0.08
    kp_ang = 0.6
    kd_lin = 0
    kd_ang = 0
    ki_lin = 0.0
    ki_ang = 0.00
    error_lin_prev = math.sqrt( (x_des - x_curr)**2 + (y_des - y_curr)**2  )
    error_ang_prev = math.atan2( (y_des - y_curr) , (x_des - x_curr) ) - math.atan2(y_curr - y_prev,x_curr- x_prev)
    sum_err_lin = 0
    sum_error_ang = 0
    random123 = Twist()
    pub.publish(random123)
    a123 = rospy.Time.now()
    b = a123.secs
    # t = rospy.Time.from_sec(a123.secs)
    # t = rospy.Time.from_sec(time.time())
    # e = rospy.Time.now()
    # epoch = rospy.Time()
    time_list = []
    c = []
    error_lin_list = []
    error_ang_list = []
    x_list=[]
    y_list =[]
    while not rospy.is_shutdown():
        # global x_curr, y_curr
        # a = Twist()
        # a.linear.x = -0.1
        # b = Twist()
        # b.angular.z = 3.14
        # rospy.loginfo("moving forward")
        # pub.publish(a)
        # pub.publish(b)
        # rospy.loginfo("Currently at x = %s, y = %s", x_curr,y_curr)
        error_lin = math.sqrt( (x_des - x_curr)**2 + (y_des - y_curr)**2  )
        d_error_lin = (error_lin - error_lin_prev) * freq
        error_lin_prev = error_lin
        sum_err_lin = sum_err_lin + error_lin
        vel = kp_lin * error_lin + kd_lin * d_error_lin + ki_lin* sum_err_lin
        theta1 = math.atan2(  2*(q_w*q_z) , (1 - 2*q_z*q_z  )  )

        error_ang = math.atan2( (y_des - y_curr) , (x_des - x_curr) ) - theta1
        
        # 2*math.atan(y / (  x+math.sqrt(x**2 + y**2) )            )

        # error_ang = math.atan2( (y_des - y_curr) , (x_des - x_curr) ) - 2*math.atan((y_curr - y_prev) / (math.sqrt((x_curr- x_prev)**2 + (y_curr - y_prev)**2) + (x_curr- x_prev)))
        d_error_ang = (error_ang - error_ang_prev) * freq
        error_ang_prev = error_ang
        sum_error_ang = sum_error_ang + error_ang
        turn = kp_ang * error_ang + kd_ang * d_error_ang + ki_ang*sum_error_ang

        a = Twist()
        a.angular.z = turn
        # if turn>0:
        #     a.angular.z = min(turn,0.5)
        # else:
        #     a.angular.z = max(turn,-0.5)
        if vel>0:
            a.linear.x = min(vel,0.3)
            # a.linear.x = vel
        else:
            a.linear.x = max(vel,-0.3)
        
        # a.linear.x = min(vel,0.3)
        # rospy.loginfo(2*math.atan((y_curr - y_prev) / (math.sqrt((x_curr- x_prev)**2 + (y_curr - y_prev)**2) + (x_curr- x_prev))))
        rospy.loginfo(error_ang)
        # now = rospy.get_rostime()
        # d = rospy.Duration.from_sec(60.1)
        d = rospy.Time.now()
        time_list.append(d.secs)
        rospy.loginfo(d.secs)
        if (len(time_list)<2):
            rospy.loginfo(time_list[0])
            c.append(time_list[0])
        else:
            rospy.loginfo(d.secs - time_list[1])
            c.append(d.secs - time_list[1])

        error_lin_list.append(error_lin)
        error_ang_list.append(error_ang)
        x_list.append(x_curr)
        y_list.append(y_curr)


        pub.publish(a)
        # rospy.is_shutdown
        if error_lin<tolerance:
            rospy.loginfo("Reached Destination")
            rospy.signal_shutdown(myhook)
            plt.figure(1)
            plt.plot(c,error_lin_list)
            plt.ylim(0,50) 
            plt.xlabel('Position Error(in meters)') 
            plt.ylabel('Time(seconds)')
            plt.title('Position Error(in meters) vs time(seconds)') 
            plt.show()


            plt.figure(2)
            plt.plot(c,error_ang_list)
            # plt.xlim(-1,100) 
            plt.title('Error in Heading angle vs time') 
            plt.show()

            plt.figure(3)
            plt.plot(x_list,y_list)
            plt.title('Path traversed by Robot')
            plt.show()

            # print time_list
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_plus_subscribe()
    except rospy.ROSInterruptException:
        rospy.loginfo("Terminating Program...")
        pass








