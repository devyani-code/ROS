#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist

def move_x(f):
    #create a new publisher. we specify the topic name, then type of message then the queue size
    pub = rospy.Publisher('/rover_diff_drive_controller/cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    vel_msg=Twist()
    input_speed=int(input('Enter speed'))
    input_direction=(input('Enter direction'))
    input_distance=f

    if (input_direction=='x'):
        vel_msg.linear.x=(input_speed)
    else:
        vel_msg.linear.y=input_speed
    vel_msg.linear.z=0
    vel_msg.angular.y=0
    vel_msg.angular.z=0
    vel_msg.angular.x=0
    while not rospy.is_shutdown():
        t0=rospy.Time.now().to_sec()
        intial_d=0
        while(intial_d<input_distance):
            pub.publish(vel_msg)
            t1=rospy.Time.now().to_sec()
            intial_d=input_speed*(t1-t0)
        if(input_direction=='x'):
            vel_msg.linear.x=0
        else:
            vel_msg.linear.y=0
        pub.publish(vel_msg)
if __name__ == '__main__':
    try:
        move_x(2)
    except rospy.ROSInterruptException:
        pass