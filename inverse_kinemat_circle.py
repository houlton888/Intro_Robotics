import matplotlib.pyplot as plt
import math
import numpy as np
import time

# create circle points
num_points = 30
L_arm = 1
# create x_vals
top_circle_x = np.arange(L_arm/2,(3/2)*L_arm,L_arm/num_points)
bottom_circle_x = np.arange((3/2)*L_arm,L_arm/2,-(L_arm/num_points))
circle_x_vals = np.append(top_circle_x,bottom_circle_x)
# create y_vals
circle_y_vals = np.zeros(num_points*2)
sign = 1
for i in range(num_points*2):
    if circle_x_vals[i] == (3/2)*L_arm: #switch to negative y vals when reach halfway
        sign = -1
    circle_y_vals[i] = sign * (math.sqrt(((-3/4)*L_arm**2)+circle_x_vals[i]*(2*L_arm-circle_x_vals[i])))

#make last val 0,0
x = np.append(circle_x_vals,L_arm/2)
y = np.append(circle_y_vals,0)

print(circle_x_vals)
print(circle_y_vals)

# Calculate theta for correcponding x,y points
points = (num_points*2)+1
theta1 = np.zeros(points)
theta2 = np.zeros(points)


for i in range(points):
    theta2[i] = -math.acos(((x[i]**2+y[i]**2)-(2*L_arm**2))/(2*L_arm**2))
    theta1[i] = -1*(math.atan2(y[i],x[i]) + math.atan2((L_arm*math.sin(theta2[i])),(L_arm+(L_arm*math.cos(theta2[i])))))

print(theta1*180/3.1415)
print(theta2*180/3.1415)
x_from_theta = np.zeros(len(x))
y_from_theta = np.zeros(len(y))
for i in range(len(x)):
    x_from_theta[i] = math.cos(theta1[i]) + math.cos(theta1[i] + theta2[i])
    y_from_theta[i] = math.sin(theta1[i]) + math.sin(theta1[i] + theta2[i])
fig, ax = plt.subplots()
ax.plot(x_from_theta,y_from_theta)
#

ax.set(xlabel='X position', ylabel='Y position', title='Arm position')
ax.grid()
plt.show(block=False)
plt.pause(10)
plt.close()