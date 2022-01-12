#!/usr/bin/env python
# license removed for brevity
##Move in x direction straight line
import rospy
from std_msgs.msg import Float64
import math

def cylinder_rotate():
    pub=rospy.Publisher("/rover_head_controller/command", Float64 ,queue_size=10)
    rospy.init_node('move_cylinder',anonymous=True)
    rate=rospy.Rate(10)
    angle=float(input('in degrees '))
    rad=angle*(math.pi)/180
    i=0
    while not rospy.is_shutdown():
        while i<rad:
            pub.publish(i)
            rate.sleep()
            i+=0.01


if __name__ == '__main__':
    try:
        cylinder_rotate()
    except rospy.ROSInterruptException:
        pass
