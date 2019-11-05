#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
import math
from std_msgs.msg import Bool
from std_msgs.msg import Float32
import message_filters

# Attributes
translationStep = 0.01
rotationStep = 0.01

targetPosition = Pose()
gpsPub = rospy.Publisher('gpsToVREP', Pose, queue_size=100)

# Initial weights for the neural network
# These weights will be changed dynamically by correlation based learning rule as a part of RNN
w11 = 0.5
w22 = 0.5
w12 = -0.5
w21 = -0.5
# The following weights will remain unchanged
w13 = 1
w23 = -1
w16 = 1
w27 = 1
w64 = 0.5
w74 = 0.5
w45 = 8
wBias5 = -3


# required for mapping the sensory outputs to a range of -1 to +1
maxDetectableDistance = 0.64 # range + radius + margin of error parameters of proximity sensors used in V-REP
minDistanceConsidered = 0.2 # very close to the obstacle, closer than this automatically sets the resultProximitySensorLeft to 1
inputStart = 1.0 / maxDetectableDistance
inputEnd = 1.0 / minDistanceConsidered
outputStart = -1.0
outputEnd = 1.0
slope = (outputEnd - outputStart) / (inputEnd - inputStart)
continueMovement = 1


def callback(gps, proximitySensorLeftBool, proximitySensorRightBool, proximitySensorLeftDistance, proximitySensorRightDistance):
    targetPosition = gps
    print targetPosition
    print "Left sensor trigger " + str(proximitySensorLeftBool.data)
    print "Right sensor trigger " + str(proximitySensorRightBool.data)
    print "Left sensor distance " + str(proximitySensorLeftDistance.data)
    print "Right sensor distance " + str(proximitySensorRightDistance.data)

    # Move the quadricopter forward 
    # This forward movement will be required later
    global continueMovement
    targetPosition.position.x += translationStep * math.cos(targetPosition.orientation.z) * continueMovement
    targetPosition.position.y += translationStep * math.sin(targetPosition.orientation.z) * continueMovement

    # IMPLEMENT NEURAL CONTROL HERE
    
    # We map the sensory output to a range of -1 to +1    
    resultProximitySensorLeft = -1 # If no obstacle is detected
    if proximitySensorLeftBool.data:
        continueMovement = 0
        if proximitySensorLeftDistance.data >= minDistanceConsidered:
            resultProximitySensorLeft = 1.0 / proximitySensorLeftDistance.data
            resultProximitySensorLeft = outputStart + slope * (resultProximitySensorLeft - inputStart)
        else:
            resultProximitySensorLeft = 1

    resultProximitySensorRight = -1 # If no obstacle is detected
    if proximitySensorRightBool.data:
        continueMovement = 0
        if proximitySensorRightDistance.data >= minDistanceConsidered:
            resultProximitySensorRight = 1.0 / proximitySensorRightDistance.data
            slope = (outputEnd - outputStart) / (inputEnd - inputStart)
            resultProximitySensorRight = outputStart + slope * (resultProximitySensorRight - inputStart)
        else:
            resultProximitySensorRight = 1



    ###############################################


    gpsPub.publish(targetPosition)

def controller():
    rospy.init_node('connectionToVREP', anonymous=True)
    gpsSub = message_filters.Subscriber('gpsToROS', Pose)
    proximitySensorLeftBoolSub = message_filters.Subscriber('proximitySensorLeftBool', Bool)
    proximitySensorRightBoolSub = message_filters.Subscriber('proximitySensorRightBool', Bool)
    proximitySensorLeftDistanceSub = message_filters.Subscriber('proximitySensorLeftDistance', Float32)
    proximitySensorRightDistanceSub = message_filters.Subscriber('proximitySensorRightDistance', Float32)
    
    approximateTimeSyncronizer = message_filters.ApproximateTimeSynchronizer([gpsSub, proximitySensorLeftBoolSub, proximitySensorRightBoolSub, proximitySensorLeftDistanceSub, proximitySensorRightDistanceSub], queue_size=10, slop=0.1, allow_headerless=True)
    approximateTimeSyncronizer.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass