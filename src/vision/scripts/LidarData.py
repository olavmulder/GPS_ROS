import csv
from ctypes import sizeof
from os import pipe
import matplotlib.pyplot as plt
import numpy as np
import struct
#import scipy.spatial.transform import Rotation as R
import scipy.spatial.transform.rotation as R
import math
import time

INPUT_CSV_DIR = "C:/Users/Olav/Documents/School/TI04/Afstudeerstage/Data van WUR/AppleSpil/"
INPUT_CSV_DIR2 = "D:/school/Afstudeerstage/Data van WUR/InHolland_rij5-2takkers-1/_r2000_node_scan/"
INPUT_CSV_DIR3 = "D:/school/Afstudeerstage/Data van WUR/InHolland_Rij6_spil_1/_r2000_node_scan/"

class lidarToPly():
    lat = 0.0
    long = 0.0
    alt = 0.0

    def setLatLongAlt(self, gpslat, gpslong, gpsalt):
        # The first
        self.latitude = gpslat
        self.longitude = gpslong
        self.altitude = gpsalt

    def __init__(self):
        return

    def hoekverdraaiing(self):
        thetaList = []
        for i in range(1440):
            thetaList.append((-np.pi / 2 ) + ( 2 * np.pi / 1439 * i))
        return thetaList

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

    def rotatePointCloud(self, csvReader):
        #get 1 line of data
        previouslat = 0
        previouslong = 0
        previousalt = 0

        xs = []
        ys = []
        zs = []

        for line in csvReader:
            if (line != csvReader[0]):

                lat = float(line[3])
                long = float(line[4])
                alt = float(line[5])

                #calculate angle between current line and previous line
                pitch, yaw, roll = self.getRotationAngle([previouslat, previouslong, previousalt], [lat, long, alt])
                x, y = self.rangeFilter(line)
                z = np.array([0] * x.size)
                w = np.array([0] * x.size)
                for i in range(x.size):
                    if (i == 0):
                        arrayLine = np.array([[x[i], y[i], z[i]]])
                        #arrayLine = np.array([[x[i], y[i], z[i], w[i]]])
                    else:
                        arrayLine = np.append(arrayLine, [[x[i], y[i], z[i]]], axis=0)
                        #arrayLine = np.append(arrayLine, [[x[i], y[i], z[i], w[i]]], axis=0)
                #rotate current line
                #r = R.Rotation.from_euler('xyz',(0, 0, 0), degrees=True) #pitch, yaw, roll
                if(pitch == 0.0 and yaw == 0.0 and roll == 0):
                    rotatedPoints = arrayLine

                    for point in rotatedPoints:
                        xs.append(self.gpsLongToMeter(self.latitude, self.longitude, long) + point[0])
                        ys.append(self.altitude - alt + point[1])
                        zs.append(self.gpsLatToMeter(self.latitude, lat) + point[2])
                else:
                    #r = R.Rotation.from_quat([pitch, yaw, roll, 0]) #pitch, yaw, roll, w
                    r = R.Rotation.from_euler('xyz',(0, 0, 0), degrees=True) #pitch, yaw, roll
                    #R.Rotation.from_quat()
                    #rotatedPoints = list of (x,y,z)
                    rotatedPoints = r.apply(arrayLine) #Rotated points

                    for point in rotatedPoints:
                        xs.append(self.gpsLongToMeter(self.latitude, self.longitude, long) + point[0])
                        ys.append(self.altitude - alt + point[1])
                        zs.append(self.gpsLatToMeter(self.latitude, lat) + point[2])

                #if (line == csvReader[400]):
                #    fig = plt.figure()
                #    ax = fig.add_subplot(111, projection='3d')
                #    ax.scatter(xs, ys, zs)
                #    plt.show()
                #    print("wait")

                print("Done with line " + line[0] + " of 16967")
                
                previouslat = float(line[3])
                previouslong = float(line[4])
                previousalt = float(line[5])

                #if (line[0] == csvReader[1000][0]):
                #    break
            else:
                previouslat = float(line[3])
                previouslong = float(line[4])
                previousalt = float(line[5])

        #move current line to correct gps coords

        return xs, ys, zs


    def pol2cart(self, theta, rho):
        x = rho * np.cos(theta)
        y = rho * np.sin(theta)
        return(x, y)

    def rangeFilter(self, csvReader):
        #calculates x and y value of each point in csvReader line
        #returns a list of x and y
        theta = self.hoekverdraaiing()
        #counter = 0
        theta1 = []
        rho = []
        for i in range(1440):
            if (float(csvReader[(i+9)]) < 3.5):
                theta1.append(float(theta[i]))
                rho.append(float(csvReader[i+9]))
        x, y = self.pol2cart(theta1, rho)
        #print(x, y)
        #counter += 1
        #plotGraph(x, y, counter)
        return x, y
        
    def plotGraph(self, x, y, counter):
        if(counter == 1300):
            plt.plot(x, y, 'b-')
            plt.show()
        return

    def readCSV(self):
        csvfile = INPUT_CSV_DIR2 + "laserscanR2000.csv"
        csvReader = []
        with open(csvfile, newline='') as csvfile:
            csv_reader  = csv.reader(csvfile, delimiter=',')
            for row in csv_reader:
                csvReader.append(row)
        csvReader.pop(0)
        return csvReader

    def write_pointcloud(self, filename, x, y, z, rgb_points=None):

        """ creates a .pkl file of the point clouds generated
        """

        arr = np.array(x)
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
        fid.write(bytes('element vertex %d\n'%len(x), 'utf-8'))
        fid.write(bytes('property float x\n', 'utf-8'))
        fid.write(bytes('property float y\n', 'utf-8'))
        fid.write(bytes('property float z\n', 'utf-8'))
        fid.write(bytes('property uchar red\n', 'utf-8'))
        fid.write(bytes('property uchar green\n', 'utf-8'))
        fid.write(bytes('property uchar blue\n', 'utf-8'))
        fid.write(bytes('end_header\n', 'utf-8'))

        # Write 3D points to .ply file
        for i in range(len(x)):
            modulo = z[i]%5
            #print(int(modulo / 2.5 * 255))
            #print(int(255 - (modulo / 2.5 * 255)))
            if modulo < -2.5:
                fid.write(bytearray(struct.pack("fffccc",x[i],y[i],z[i],
                                                r_points[i].tobytes(),int(255 - ((-1 * modulo - 2.5) / 2.5 * 255)).to_bytes(1, 'little'),
                                                b_points[i].tobytes())))
            elif modulo < 0:
                fid.write(bytearray(struct.pack("fffccc",x[i],y[i],z[i],
                                                int(-1 * modulo / 2.5 * 255).to_bytes(1, 'little'),g_points[i].tobytes(),
                                                b_points[i].tobytes())))
            elif modulo < 2.5: 
                fid.write(bytearray(struct.pack("fffccc",x[i],y[i],z[i],
                                                int(modulo / 2.5 * 255).to_bytes(1, 'little'),g_points[i].tobytes(),
                                                b_points[i].tobytes())))
            else:
                fid.write(bytearray(struct.pack("fffccc",x[i],y[i],z[i],
                                                r_points[i].tobytes(),int(255 - ((modulo - 2.5) / 2.5 * 255)).to_bytes(1, 'little'),
                                                b_points[i].tobytes())))
        fid.close()

    def getXYZ(self, csvReader):
        x = [] 
        y = []
        z = []
        counter = 0
        for i in csvReader:
            tempX, tempZ = self.rangeFilter(i)
            lat = float(i[3])
            long = float(i[4])
            alt = float(i[5])
            for item in tempX:
                x.append(self.gpsToMeter(self.long, long) + item)
                z.append(self.gpsToMeter(self.lat, lat))
            for item in tempZ:
                y.append(self.alt - alt + item)
        return x, y, z

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

    def saveGpsData(self, csvReader):
        with open('gpsDatapeer2.txt', 'w') as f:
            for item in csvReader:
                f.write("%s, " % item[3])
                f.write("%s\n" % item[4])

    def main(self): 
        csvReader = self.readCSV()
        self.setLatLongAlt(float(csvReader[0][3]), float(csvReader[0][4]), float(csvReader[0][5]))
        startTime = time.perf_counter()
        x, y, z = self.rotatePointCloud(csvReader)
        endTime = time.perf_counter()
        totalTime = endTime - startTime
        print(totalTime)
        #x, y, z = self.getXYZ(csvReader)
        #print(x, y, z)
        print(len(x))
        self.write_pointcloud('testing1.ply', x, y, z)
        #self.saveGpsData(csvReader)
        #self.checkDistanceBetweenLines(csvReader)

if __name__ == '__main__':
    ply = lidarToPly()
    ply.main()