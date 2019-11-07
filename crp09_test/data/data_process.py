import numpy as np
import matplotlib.pyplot as plt
import re

# read files
origin_file = open('joint_states.txt','r')
position_file = open('position_data.txt','w')
lines = origin_file.readlines()
origin_file.close()

# get position datas and put into a list "data"
data = []
for line in lines:
   if "position" in line:
		position_file.write(line)
		data.append(re.findall(r"\-?\d+\.?\d*",line))
position_file.close()
print('All the position data number is:', len(data))

# get each joints' position data
joint1_postion = []
joint2_postion = []
joint3_postion = []
joint4_postion = []
joint5_postion = []
joint6_postion = []
finger_joint1 = []
finger_joint2 = []
for i in range(len(data)):  # i is from 0 to data number -1
	joint1_postion.append(float(data[i][0]))
	joint2_postion.append(float(data[i][1]))
	joint3_postion.append(float(data[i][2]))
	joint4_postion.append(float(data[i][3]))
	joint5_postion.append(float(data[i][4]))
	joint6_postion.append(float(data[i][5]))
	finger_joint1.append(float(data[i][6]))
	finger_joint2.append(float(data[i][7]))

# PLOT A FIGURE BY MATPLOTLIB
t = np.linspace(0, 10, len(data))
plt.xlabel("move time t") 
plt.ylabel("joint postion arc") 

plt.subplot(2,3,1)
plt.title("joint 1") 
plt.plot(t, joint1_postion)

plt.subplot(2,3,2)
plt.title("joint 2") 
plt.plot(t, joint2_postion)

plt.subplot(2,3,3)
plt.title("joint 3") 
plt.plot(t, joint3_postion)

plt.subplot(2,3,4)
plt.title("joint 4") 
plt.plot(t, joint4_postion)

plt.subplot(2,3,5)
plt.title("joint 5") 
plt.plot(t, joint5_postion)

plt.subplot(2,3,6)
plt.title("joint 6") 
plt.plot(t, joint6_postion)

plt.show()