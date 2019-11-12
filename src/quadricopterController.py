#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
import math
from std_msgs.msg import Bool
from std_msgs.msg import Float32
import message_filters


# Attributes
translationStep = 0.01
rotationStep = 0.02

targetPosition = Pose()
gpsPub = rospy.Publisher('gpsToVREP', Pose, queue_size=100)

a1 = -1
a2 = -1
# Initial weights for the neural network
# These weights will be changed dynamically by correlation based learning rule as a part of RNN
w11T = 0.5
w11TPlusOne = 0.5
w22T = 0.5
w22TPlusOne = 0.5
w12T = -0.5
w12TPlusOne = -0.5
w21TPlusOne = -0.5
w21T = -0.5
# The following weights will remain unchanged
w13 = -1
w23 = 1
w16 = -1
w27 = -1
w64 = 0.5
w74 = 0.5
w45 = 8
wBias5 = -3
bias = 1

# constants for correlation based learning
muR = 0.01
muQ = 0.015
gamma = 0.0003
k = -0.01

v1TMinusOne = 0
v2TMinusOne = 0
q1T = 1
q2T = 1

o1TMinusOne = -1
o2TMinusOne = -1

# required for mapping the sensory outputs to a range of -1 to +1
maxDetectableDistance = 0.64 # range + radius + margin of error parameters of proximity sensors used in V-REP
minDistanceConsidered = 0.2 # very close to the obstacle, closer than this automatically sets the o1 to 1
distanceAverage = (minDistanceConsidered + maxDetectableDistance) / 2
# inputStart = 1.0 / maxDetectableDistance
# inputEnd = 1.0 / minDistanceConsidered
# outputStart = -1.0
# outputEnd = 1.0
# slope = (outputEnd - outputStart) / (inputEnd - inputStart)

def reflex(sensoryInput):
    return 1 if sensoryInput > -0.5 else 0

def sigmoid(z):
    return 1 / (1 + math.exp(-z))

def callback(gps, proximitySensorLeftBool, proximitySensorRightBool, proximitySensorLeftDistance, proximitySensorRightDistance):

    # getting the values of global variables
    # global slope
    global w11T
    global w12T
    global w22T
    global w21T
    global w11TPlusOne
    global w22TPlusOne
    global w12TPlusOne
    global w21TPlusOne
    global v1TMinusOne
    global v2TMinusOne
    global q1T
    global q2T
    global o1TMinusOne
    global o2TMinusOne
    global a1
    global a2


    targetPosition = gps
    # print targetPosition
    # print "Left sensor trigger " + str(proximitySensorLeftBool.data)
    # print "Right sensor trigger " + str(proximitySensorRightBool.data)
    print "Left sensor distance " + str(proximitySensorLeftDistance.data)
    print "Right sensor distance " + str(proximitySensorRightDistance.data)

    # IMPLEMENT NEURAL CONTROL HERE
    # We map the sensory output to a range of -1 to +1    
    o1 = -1 # If no obstacle is detected
    if proximitySensorLeftBool.data:
        if proximitySensorLeftDistance.data >= minDistanceConsidered:
            # o1 = 1.0 / proximitySensorLeftDistance.data
            # o1 = outputStart + slope * (o1 - inputStart)
            o1 = math.tanh((distanceAverage - proximitySensorLeftDistance.data) * 25)
        else:
            o1 = 1

    o2 = -1 # If no obstacle is detected
    if proximitySensorRightBool.data:
        if proximitySensorRightDistance.data >= minDistanceConsidered:
            # o2 = 1.0 / proximitySensorRightDistance.data
            # slope = (outputEnd - outputStart) / (inputEnd - inputStart)
            # o2 = outputStart + slope * (o2 - inputStart)
            o2 = math.tanh((distanceAverage - proximitySensorRightDistance.data) * 25)
        else:
            o2 = 1

    v1T = (o1 + 1) / 2
    v2T = (o2 + 1) / 2
    r1 = reflex(o1)
    r2 = reflex(o2)

    # Weights for synaptic plasticity
    w11TPlusOne = w11T + (muR * v1TMinusOne * v1T * r1) + (gamma * (k - v1T) * w11T * w11T)
    w22TPlusOne = w22T + (muR * v2TMinusOne * v2T * r2) + (gamma * (k - v2T) * w22T * w22T)

    a1 = math.tanh(w11T * math.tanh(a1) + w12T * math.tanh(a2) + 7 * o1)
    a2 = math.tanh(w21T * math.tanh(a1) + w22T * math.tanh(a1) + 7 * o2)

    o3 = a1 * w13 + a2 * w23
    a3 = math.tanh(o3)
    targetPosition.orientation.z += a3 * rotationStep

    # Pitch Control
    q1TPlusOne = muQ * v1TMinusOne * v1T * r1 + gamma * (k - v1T) * q1T * q1T
    q2TPlusOne = muQ * v2TMinusOne * v2T * r2 + gamma * (k - v2T) * q2T * q2T

    w12TPlusOne = w12T + 0.5 * (q1TPlusOne + q2TPlusOne)
    w21TPlusOne = w12TPlusOne
    a6 = sigmoid(w16 * o1)
    a7 = sigmoid(w27 * o2)
    a4 = math.tanh(w64 * a6 + w74 * a7)
    a5 = math.tanh(w45 * a4 +  wBias5 * bias)
    
    print "r1 = " + str(r1)
    print "r2 = " + str(r2)
    print "w11 = " + str(w11T)
    print "w22 = " + str(w22T)
    print "w12 = " + str(w12T)
    print "o1 = " + str(o1)
    print "o2 = " + str(o2)
    print "a1 = " + str(a1)
    print "a2 = " + str(a2)
    print "a3 = " + str(a3)
    print "a5 = " + str(a5)
    print

    # Assigning values for the next iteration
    o1TMinusOne = o1
    o2TMinusOne = o2
    v1TMinusOne = v1T
    v2TMinusOne = v2T
    q1T = q1TPlusOne
    q2T = q2TPlusOne
    w11T = w11TPlusOne
    w22T = w22TPlusOne
    w12T = w12TPlusOne
    w21T = w21TPlusOne

    targetPosition.position.x += translationStep * math.cos(targetPosition.orientation.z) * a5
    targetPosition.position.y += translationStep * math.sin(targetPosition.orientation.z) * a5
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