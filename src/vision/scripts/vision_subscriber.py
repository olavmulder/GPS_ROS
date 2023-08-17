#! /usr/bin/python3
from ctypes import sizeof
from os import pipe
import matplotlib.pyplot as plt
import numpy as np
import struct
from pandas import array
#import scipy.spatial.transform import Rotation as R
import scipy.spatial.transform.rotation as R
import math
import time
import rospy
import sensor_msgs.point_cloud2
from vision.msg import visionMessage


class lidarToPly():
    lat = 0.0
    long = 0.0
    alt = 0.0
    pt_x = []
    pt_y = []
    pt_z = []
    previouslat = 0
    previouslong = 0
    previousalt = 0
    iterations = 0
    def setLatLongAlt(self, gpslat, gpslong, gpsalt):
        # The first
        self.latitude = gpslat
        self.longitude = gpslong
        self.altitude = gpsalt
    def setPointCloud(self, pcFromMsg):
        for point in sensor_msgs.point_cloud2.read_points(pcFromMsg, skip_nans=True):
            self.pt_x = point[0]
            self.pt_y = point[1]
            self.pt_z = point[2]

    def __init__(self):
        return

    def getRotationAngle(self, coordA, coordB):
        z = self.gpsLatToMeter(coordB[0], coordA[0])#diepte lat noord zuid
        x = self.gpsLongToMeter(self.latitude, coordB[1], coordA[1])#breedte long oost west
        y = coordB[2] - coordA[2]#hoogte alt
        #
        #Yaw: lat and long
        yaw = 180* math.atan2(z, x)/np.pi
        #Pitch: lat and long for distance and alt to get angle over distance
        distance = np.lib.scimath.sqrt((z*z) + (x*x))
        pitch = 180* math.atan2(y, distance)/np.pi

        return pitch, yaw, 0.0

    


    def rotatePointCloud(self):
        #get 1 line of data
        xs = self.pt_x
        ys = self.pt_y
        zs = self.pt_z
        
        arrayLine = []
        if self.iterations != 0:
            pitch, yaw, roll = self.getRotationAngle([self.previouslat, self.previouslong, self.previousalt], [self.lat, self.long, self.alt])

            self.lat = self.latitude
            self.long = self.longitude
            self.alt = self.altitude

           

            '''x, y = self.rangeFilter(xs, ys, zs)
            z = np.array([0] * x.size)
            w = np.array([0] * x.size)'''
            for i in range(len(xs)):
                if (i == 0):
                    arrayLine = np.array([[xs[i], ys[i], zs[i]]])
                    #arrayLine = np.array([[x[i], y[i], z[i], w[i]]])
                else:
                    arrayLine = np.append(arrayLine, [[xs[i], ys[i], zs[i]]], axis=0)
                    #arrayLine = np.append(arrayLine, [[x[i], y[i], z[i], w[i]]], axis=0)
            #rotate current line
            #r = R.Rotation.from_euler('xyz',(0, 0, 0), degrees=True) #pitch, yaw, roll'''
            if(pitch == 0.0 and yaw == 0.0 and roll == 0):
                rotatedPoints = arrayLine
                for point in rotatedPoints:
                    xs.append(self.gpsLongToMeter(self.latitude, self.longitude, self.long) + point[0])
                    ys.append(self.altitude - self.alt + point[1])
                    zs.append(self.gpsLatToMeter(self.latitude, self.lat) + point[2])
            else:
                #r = R.Rotation.from_quat([pitch, yaw, roll, 0]) #pitch, yaw, roll, w
                r = R.Rotation.from_euler('xyz',(0, 0, 0), degrees=True) #pitch, yaw, roll
                #R.Rotation.from_quat()
                #rotatedPoints = list of (x,y,z)
                rotatedPoints = r.apply(arrayLine) #Rotated points

                for point in rotatedPoints:
                    xs.append(self.gpsLongToMeter(self.latitude, self.longitude, self.long) + point[0])
                    ys.append(self.altitude - self.alt + point[1])
                    zs.append(self.gpsLatToMeter(self.latitude, self.lat) + point[2])

                self.previouslat = self.lat
                self.previouslong = self.long
                self.previousalt = self.alt
        else:
            self.previouslat = self.lat
            self.previouslong = self.long
            self.previousalt = self.alt
            
        return xs, ys, zs


    

    def gpsToMeter(self, v1, v2):
        r = 6371000 #straal aarde in meter
        #print(v1, v2)
        return (2 * np.pi * r * ((v2 - v1) / 360))

    def gpsLatToMeter(self, v1, v2):
        r = 6371000 #straal aarde in meter
        #print(v1, v2)
        return (2 * np.pi * r * ((v2 - v1) / 360))

    def gpsLongToMeter(self, angle, v1, v2):
        r = 6371000 #straal earth in meters
        #print(angle)
        #print(math.cos(math.radians(angle)))
        a = (math.cos(math.radians(angle))) * r #Hight rechthoekige triangle, needed to calculate radius
        #print(a)
        #a = math.sqrt((r * r) - (x * x)) #radius circle at given coordinates
        meters = 2 * np.pi * a * ((v2 - v1) / 360) #
        #print(meters)
        return meters



    def write_pointcloud(self, filename, xList, yList, zList, rgb_points=None):

        """ creates a .pkl file of the point clouds generated
        """
        lenX = len(xList)
        arr = np.array(xList)
        #assert xyz_points.shape[1] == 3,'Input XYZ points should be Nx3 float array'
        if rgb_points is None:
            r_points = np.ones(arr.shape).astype(np.uint8)*255
            g_points = np.ones(arr.shape).astype(np.uint8)*255
            b_points = np.ones(arr.shape).astype(np.uint8)*0
        #assert xyz_points.shape == rgb_points.shape,'Input RGB colors should be Nx3 float array and have same size as input XYZ points'

        # Write header of .ply file
        fid = open(filename,'wb')
        fid.write(bytes('ply\n', 'utf-8'))
        fid.write(bytes('format binary_little_endian 1.0\n', 'utf-8'))
        #fid.write(bytes('element vertex %d\n'%len(x), 'utf-8'))
        fid.write(bytes('property float x\n', 'utf-8'))
        fid.write(bytes('property float y\n', 'utf-8'))
        fid.write(bytes('property float z\n', 'utf-8'))
        fid.write(bytes('property uchar red\n', 'utf-8'))
        fid.write(bytes('property uchar green\n', 'utf-8'))
        fid.write(bytes('property uchar blue\n', 'utf-8'))
        fid.write(bytes('end_header\n', 'utf-8'))

        # Write 3D points to .ply file
        for i in range(lenX):
            modulo = zList[i]%5
            #print(int(modulo / 2.5 * 255))
            #print(int(255 - (modulo / 2.5 * 255)))
            if modulo < -2.5:
                fid.write(bytearray(struct.pack("fffccc",xList[i],yList[i],zList[i],
                                                r_points[i].tobytes(),int(255 - ((-1 * modulo - 2.5) / 2.5 * 255)).to_bytes(1, 'little'),
                                                b_points[i].tobytes())))
            elif modulo < 0:
                fid.write(bytearray(struct.pack("fffccc",xList[i],yList[i],zList[i],
                                                int(-1 * modulo / 2.5 * 255).to_bytes(1, 'little'),g_points[i].tobytes(),
                                                b_points[i].tobytes())))
            elif modulo < 2.5: 
                fid.write(bytearray(struct.pack("fffccc",xList[i],yList[i],zList[i],
                                                int(modulo / 2.5 * 255).to_bytes(1, 'little'),g_points[i].tobytes(),
                                                b_points[i].tobytes())))
            else:
                fid.write(bytearray(struct.pack("fffccc",xList[i],yList[i],zList[i],
                                                r_points[i].tobytes(),int(255 - ((modulo - 2.5) / 2.5 * 255)).to_bytes(1, 'little'),
                                                b_points[i].tobytes())))
        fid.close()

    def ReadData(self,data):
        rospy.loginfo(rospy.get_caller_id() + "gpsWidth: %s", data.gpslat)
        rospy.loginfo(rospy.get_caller_id() + "gpsLength: %s", data.gpslon)
        rospy.loginfo(rospy.get_caller_id() + "gpsHight: %s", data.gpsheight)

        self.setLatLongAlt(data.gpslat, data.gpslon, data.gpsheight)
        self.setPointCloud(data.pc)
        x,y,z = self.rotatePointCloud()

        self.write_pointcloud(str(self.iterations)+'.ply', x, y, z)
        self.iterations += 1
    def main(self):
        
        rospy.init_node('GetDataVision', anonymous=True)
        rospy.Subscriber('pc_label', visionMessage, self.ReadData)
        rospy.spin()

if __name__ == '__main__':
    ply = lidarToPly()
    ply.main()