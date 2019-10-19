#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
import math

# Attributes
translationStep = 0.01
rotationStep = 0.01

targetPosition = Pose()
gpsPublisher = rospy.Publisher('gpsToVREP', Pose, queue_size=100)
def gpsCallback(msg):
    targetPosition = msg
    print targetPosition

    # Move the quadricopter forward (currently this is not working as intended)
    # This forward movement will be required later
    targetPosition.position.x += translationStep * math.cos(targetPosition.orientation.z)
    targetPosition.position.x += translationStep * math.sin(targetPosition.orientation.z)

    # IMPLEMENT NEURAL CONTROL HERE

    ###############################################


    gpsPublisher.publish(targetPosition)
    print targetPosition

def controller():
    rospy.init_node('connectionToVREP', anonymous=True)
    gpsSubscriber = rospy.Subscriber('gpsToROS', Pose, gpsCallback)
    rospy.spin()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass