#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from gps_data.msg import gps

last_gps = None

gpsPublisher = rospy.Publisher('PC_lables', gps, queue_size = 10)

def neuvition_callback(data): 
    rospy.loginfo("Received cloud")
    
    if last_gps != None:
        last_gps.pc = data
        #rospy.loginfo("DATA: %s", last_gps)
        gpsPublisher.publish(last_gps)



def gps_callback(data):
    global last_gps
    last_gps = data
    rospy.loginfo("Received GPS")

def main():
    
    rospy.init_node("PC_labels", anonymous=False)
    rospy.Subscriber("neuvition_cloud", PointCloud2, neuvition_callback)
    rospy.Subscriber("gps_data", gps, gps_callback)
    rospy.spin()

if __name__ == "__main__":
    main()