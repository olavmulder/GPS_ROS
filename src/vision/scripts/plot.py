#!/usr/bin/env python3
import rospy
import matplotlib.pyplot as plt
import numpy as np

import sys
value = []
labels = []

count = 0
i = 0
text = sys.argv[1]
f = open(text, "r")
lines = f.readlines()
for line in lines:
    
    num = float(line.strip())
    if num != 0 :
        
        value.append(num)
        labels.append(count)
        print(count, " ", num)
        count+=1
    i+=1

width = 0.2      # the width of the bars: can also be len(x) sequence

formatted = "{:.2f}".format(count/i*100)

fig, ax = plt.subplots()

ax.bar(labels, value, width, label="1")
ax.set_ylabel('distance in meter')
ax.set_xlabel('number')
ax.set_title('total of ' + str(i) + ' gps points, '+ str(count) +' points where not zero '+ formatted+ "%")
plt.show()

'''
plt.plot(labels, value, 'ro')
plt.ylabel('some numbers')
plt.show()
'''
