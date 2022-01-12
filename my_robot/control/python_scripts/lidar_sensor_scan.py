#!/usr/bin/env python
# license removed for brevity
##Move in x direction straight line
import rospy
from sensor_msgs import msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class avoid_obstacle():

    def __init__(self):
        rospy.init_node('avoid_obstacle',anonymous=True)
        self.pub=rospy.Publisher('/rover_diff_drive_controller/cmd_vel',Twist,queue_size=10)
        self.sub = rospy.Subscriber('/macroed/laser/scan', LaserScan, self.callback)
        self.laser=LaserScan()
        self.vel_msg=Twist()
        self.rate = rospy.Rate(10)
        self.range_values=[]
    def callback(self,msg):
        
    def avoid(self):
        print(self.laser.ranges)
        angular_speed=45
        angle=90
        angular_speed=angular_speed*3.14/180
        angle=angle*3.14/180
        self.vel_msg.linear.y=0
        self.vel_msg.linear.z=0
        self.vel_msg.angular.x=0
        self.vel_msg.angular.y=0
        self.vel_msg.linear.x=0
        for i in range(0,len(self.laser.ranges)):
            if (i<1.8):
                self.vel_msg.angular.z=angular_speed
                while not rospy.is_shutdown():
                    t0=rospy.Time.now().secs
                    current_angle=0
                    while(current_angle<=angle):
                        self.pub.publish(self.vel_msg)
                        t1=rospy.Time.now().secs
                        current_angle=0.4*angular_speed*(t1-t0)
                        self.rate.sleep()
                    else:
                        self.vel_msg.angular.z=0
                        self.pub.publish(self.vel_msg)
                        break
if __name__ == '__main__':
    try:
        x=avoid_obstacle()
        x.avoid()
    except rospy.ROSInterruptException:
        pass

    


   
   




# msg.ranges is a list of length 720 at i=360 the object at 90 degree is shown