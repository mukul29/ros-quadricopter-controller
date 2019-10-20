#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
import math
from std_msgs.msg import Bool
import message_filters

# Attributes
translationStep = 0.01
rotationStep = 0.01

targetPosition = Pose()
gpsPub = rospy.Publisher('gpsToVREP', Pose, queue_size=100)
def callback(gps, proximitySensorLeft, proximitySensorRight):
    targetPosition = gps
    print targetPosition
    print proximitySensorLeft
    print proximitySensorRight

    # Move the quadricopter forward 
    # This forward movement will be required later
    targetPosition.position.x += translationStep * math.cos(targetPosition.orientation.z)
    targetPosition.position.y += translationStep * math.sin(targetPosition.orientation.z)

    # IMPLEMENT NEURAL CONTROL HERE

    ###############################################


    gpsPub.publish(targetPosition)
    print targetPosition

def controller():
    rospy.init_node('connectionToVREP', anonymous=True)
    gpsSub = message_filters.Subscriber('gpsToROS', Pose)
    proximitySensorLeftSub = message_filters.Subscriber('proximitySensorLeft', Bool)
    proximitySensorRightSub = message_filters.Subscriber('proximitySensorRight', Bool)

    approximateTimeSyncronizer = message_filters.ApproximateTimeSynchronizer([gpsSub, proximitySensorLeftSub, proximitySensorRightSub], queue_size=10, slop=0.1, allow_headerless=True)
    approximateTimeSyncronizer.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass