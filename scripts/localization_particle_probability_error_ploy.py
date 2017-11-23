#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import argparse
import rospy
import rostopic
import std_msgs.msg
from geometry_msgs.msg import Point
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32

class ScatterPlot():
    
    sub_point = Point()

    def euclidean_dist(self, particle):
        dist = ((self.sub_point.x - particle.x)**2 + (self.sub_point.y - particle.y)**2 + (self.sub_point.z - particle.z)**2)**0.5
        return dist

    def update_location(self, msg):
        sub_point = msg.point




    def plot(self, msg):
        
        rospy.loginfo("Plotting Update")

        error_data = []
        for particle in msg.points:
            error_data.append(self.euclidean_dist(particle))

        plt.scatter(x = msg.channels[0].values, y = error_data)

        plt.ylabel('Error')
        
        plt.xlabel('Weight')
        plt.title('Weight vs. Error Scatter Plot')

        plt.legend()

        plt.pause(0.05)
        plt.gca().clear()
         

    def __init__(self):

        # Subscribe to the topic
        self.sub = rospy.Subscriber("/localization/particles", PointCloud, self.plot)

        # Subscribe to the topic
        self.sub = rospy.Subscriber("/simulator/cobalt/position", PointStamped, self.update_location)

        rospy.loginfo("Subscribed to /localization/particles")

        plt.ion()
        plt.show()


    def __del__(self):
        self.sub.unregister()


if __name__ == "__main__":
    global h

    rospy.init_node("ScatterPlot", anonymous=True)

    h = ScatterPlot()

    rospy.spin()
