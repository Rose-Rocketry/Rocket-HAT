import random
import os
import numpy as np
import math

#file = open("datafile-12.dat", "r")
#a = str(file.read())
f = np.fromfile(open("datafile-2.dat"), dtype = np. float32)
'''
# this an array of strings
dot = '.'
bi_dot = dot.encode('utf-8')
bi_angle = a.split(dot)

la = len(bi_angle)
print(la)
# loop through the array and convert
int_angle = []
for bi in bi_angle:
    try:
        i = int(bi)
    except ValueError:
        break
    int_angle.append(i)


float_angle = [i/100.0 for i in int_angle]
'''

#the length of the file
n=len(f)
print(n)

#max power
Pr = np.max(f)
print(Pr)
#max power position
pos = (np.where(f == Pr))
print(pos)
int_pos = pos[0][0]
print(int_pos)


#calculate distance
Pt = -28497
c = 299792458
d = c/(math.sqrt(Pr/Pt)*4*math.pi*426000000)
print(d)

#responed angle to the power
#max_angle = float_angle[int_pos]
#print(max_angle)

A = 1.8/5
angle = (int_pos/300000000)*A/0.316
print(angle)

