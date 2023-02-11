#!/usr/bin/env python3
"This is a submission by Team KB_2198"
#
import sys
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
regions = {
        'bright':  0 ,
        'fright':  0 ,
        'front':  0 ,
        'fleft':   0 ,
        'bleft':   0
    }
lastTime_Ang, lastTime_Lin, errSum_Ang, errSum_Lin, lastErr_Ang, lastErr_Lin = [0,0,0,0,0,0]

def Compute_Ang(Input, Setpoint):
    now = rospy.get_time()
    global errSum_Ang, lastTime_Ang, errSum_Ang, lastErr_Ang
    kp = 0
    ki = 10
    kd = 0

    timeChange = (now - lastTime_Ang)
    error = Setpoint - Input
    errSum_Ang += (error * timeChange)
    dErr = (error - lastErr_Ang) / timeChange
    Output = kp * error + ki * errSum_Ang + kd * dErr
    if Output > 180:
        Output = Output % 180
    elif Output < (-180):
        Output = Output % (-180)
    lastErr_Ang = error
    lastTime_Ang = now
    return Output

def Compute_Lin(Input, Setpoint):
    now = rospy.get_time()
    global errSum_Lin, lastTime_Lin, errSum_Lin, lastErr_Lin
    kp = 0
    ki = 10
    kd = 0

    timeChange = (now - lastTime_Lin)
    error = Setpoint - Input
    errSum_Lin += (error * timeChange)
    dErr = (error - lastErr_Lin) / timeChange
    Output = kp * error + ki * errSum_Lin + kd * dErr
    lastErr_Lin = error
    lastTime_Lin = now
    return Output


def laser_callback(msg):
    "This is a callback function"
    # print (msg)
    global regions
    range_max = 3
    regions = {
        'bright':  min(min(msg.ranges[0:143]), range_max) ,
        'fright':  min(min(msg.ranges[144:287]), range_max) ,
        'front':  min(min(msg.ranges[288:431]), range_max) ,
        'fleft':   min(min(msg.ranges[432:575]), range_max) ,
        'bleft':   min(min(msg.ranges[576:719]), range_max)
    }
    # print(regions)
def control_loop():
    """Controlling function for the Krishi Bot."""
    rospy.init_node('ebot_controller')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rate = rospy.Rate(10)

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)
    start = rospy.get_time()
    stop = rospy.get_time()
    count = 0
    path = '/home/saurabh/KB_ws/src/eyrc-2022_krishibot/scripts/'
    file1 = open(path + 'data_angle1.txt','w',encoding='UTF-8')
    file2 = open(path + 'data_lin1.txt','w',encoding='UTF-8')


    while not rospy.is_shutdown():
        rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
        #
        # Your algorithm to navigate
        #
        while (stop - start) > 45:
            # Loop to stop or bot after completing the task
            velocity_msg.linear.x = Compute_Lin(velocity_msg.linear.x, 0)
            velocity_msg.linear.y = 0
            velocity_msg.angular.z = 0
            pub.publish(velocity_msg)
            rate.sleep()
            count +=1
            if count > 15:
                sys.exit()
        while regions['fleft'] < 1.2:
            # Loop to move forward taking 'fleft' as the reference.
            stop = rospy.get_time()
            if (stop - start) > 45:
                break
            velocity_msg.linear.x = Compute_Lin(velocity_msg.linear.x, 1)
            file2.write(str(velocity_msg.linear.x) + "\n\n")
            velocity_msg.linear.y = 0
            velocity_msg.angular.z = 0
            pub.publish(velocity_msg)
            rate.sleep()

        while regions['bleft'] < 0.7:
            # Loop to move forward taking 'bleft' as the reference.
            stop = rospy.get_time()
            if (stop - start) > 45:
                break
            velocity_msg.linear.x = Compute_Lin(velocity_msg.linear.x, 0.45)
            file2.write(str(velocity_msg.linear.x) + "\n\n")
            velocity_msg.linear.y = 0
            velocity_msg.angular.z = 0
            pub.publish(velocity_msg)
            rate.sleep()

        while regions['bleft'] > 2:
            # Loop to turn around taking 'bleft' as the reference for first part of the turn.
            stop = rospy.get_time()
            if (stop - start) > 45:
                break
            velocity_msg.linear.x = Compute_Lin(velocity_msg.linear.x, 0.45)
            file2.write(str(velocity_msg.linear.x) + "\n\n")
            velocity_msg.linear.y = 0
            velocity_msg.angular.z = Compute_Ang(velocity_msg.angular.z, 1)
            file1.write(str(velocity_msg.angular.z) + "\n\n")
            pub.publish(velocity_msg)
            rate.sleep()

        while regions['bleft'] < 2:
            # Loop to turn around taking 'bleft' as the reference for final part of the turn.
            stop = rospy.get_time()
            if (stop - start) > 45:
                break
            velocity_msg.linear.x = Compute_Lin(velocity_msg.linear.x, 0.45)
            file2.write(str(velocity_msg.linear.x) + "\n\n")
            velocity_msg.linear.y = 0
            velocity_msg.angular.z = Compute_Ang(velocity_msg.angular.z, 0.8)
            file1.write(str(velocity_msg.angular.z) + "\n\n")
            pub.publish(velocity_msg)
            rate.sleep()
            break


if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
