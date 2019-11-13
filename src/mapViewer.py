#!/usr/bin/env python
import rospy
import message_filters
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
import matplotlib.animation
import sys

left = []
right = []

fig = plt.figure()
ax = fig.add_subplot(111)
scatter, = ax.plot(left, right, "o", markersize=1)
ax.set_xlim(-2.5, 2.5)
ax.set_ylim(-2.5, 2.5)
ax.set_title('Obstacle Map')
ax.set_xlabel('x-->')
ax.set_ylabel('y-->')

def callback(detectedPointLeft, detectedPointRight):
    # accessing global variables
    global left 
    global right
    if detectedPointLeft.z != 0.0:
        # print detectedPointLeft
        left.append(detectedPointLeft.x)
        right.append(detectedPointRight.y)

    if detectedPointRight.z != 0.0:
        # print detectedPointRight
        left.append(detectedPointRight.x)
        right.append(detectedPointRight.y)
    
    zippedList = list(set(zip(left, right)))
    resultList = [[ i for i, j in zippedList],
                  [  j for i, j in zippedList]]
    scatter.set_data(resultList[0], resultList[1])

def dummyFunction(dummy):
    pass

def mapViewer():
    rospy.init_node('connectionToVREP2', anonymous=True)
    detectedPointLeftSub = message_filters.Subscriber('proximitySensorLeftDetectedCoordinates', Point)
    detectedPointRightSub = message_filters.Subscriber('proximitySensorRightDetectedCoordinates', Point)

    approximateTimeSyncronizer = message_filters.ApproximateTimeSynchronizer([detectedPointLeftSub, detectedPointRightSub], queue_size=10, slop=0.1, allow_headerless=True)
    approximateTimeSyncronizer.registerCallback(callback)

if __name__ == '__main__':
    try:
        mapViewer()
    except rospy.ROSInterruptException:
        pass
    ani = matplotlib.animation.FuncAnimation(fig, dummyFunction, repeat=False)
    plt.show()
    
    rospy.spin()