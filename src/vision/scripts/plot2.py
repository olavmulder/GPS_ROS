#!/usr/bin/env python3
import rospy
import matplotlib.pyplot as plt
import numpy as np

import sys
value = []
labels = []


text = sys.argv[1]
text2 = sys.argv[2]
lat = open(text, "r")
lon = open(text2, "r")
lines = lon.readlines()
line2 = lat.readlines()
for line in lines:
    
    lat = float(line.strip())
    value.append(lat)
for line in line2:
    lon = float(line.strip())
    labels.append(lon)

fig, ax = plt.subplots()

ax.plot(labels, value, 'o')
ax.set_ylabel('lat')
ax.set_xlabel('lon')
ax.set_title("GPS point with longitute & latitude")
plt.show()

'''
plt.plot(labels, value, 'ro')
plt.ylabel('some numbers')
plt.show()
'''
