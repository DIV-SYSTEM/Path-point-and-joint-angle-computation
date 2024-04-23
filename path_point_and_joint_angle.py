import time
import math
import numpy as np
import json
import numpy
from scipy.spatial.transform import Rotation as scipyRotation
from skspatial.objects import Plane
import transforms3d 
import pybullet 
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D
from pybullet_utils import bullet_client
import pandas
import platform
import random
from utils_functions import *

libraries=[time,math,numpy,json,scipyRotation,pybullet,pandas,Plane,transforms3d]

'''importing neccessary functions'''
from Kinematics.Lara.lara10_kinematics import lara10
obj=lara10()


spine_translation_pos_t6_2_t7_lara =  [-24.44120225, -9.00091956, 233.30794654] 
spine_euler_angles_t6_2_t7_lara =  [125.28248231, -16.3377963, 136.3419605]
T_6_7=get_t6_2_t7_pose_mm(spine_translation_pos_t6_2_t7_lara,spine_euler_angles_t6_2_t7_lara,libraries)


lista = []
listb = []

'''We are taking position and orientation as a user input for the pointA and pointB'''
for j in range(2):
   for i in range(6):
     user_input = input("Enter integer {i+1} for {j+1}")
     try:
        int_val= int(user_input)
        if j==0:
         lista.append(int_val)
        if j==1:
         listb.append(int_val)
     except ValueError:
         print("Invalid input. Please enter an integer.")
         i -= 1 

#lista = [100,100,100,0,0,0]
#listb = [300,200,170,0,0,0]

"""creating a list known as lop that will store all the points"""
lop=[]
lop.append(lista)

def generate_points_numpy(lista, listb):
       x1= lista[0]
       y1= lista[1]
       z1= lista[2]
       x2= listb[0]
       y2= listb[1]
       z2= listb[2]
       x_values = np.linspace(x1, x2, 100+ 2)[1:-1]
       y_values = np.linspace(y1, y2, 100+ 2)[1:-1]
       z_values = np.linspace(z1, z2, 100+ 2)[1:-1]
       for x,y,z in zip(x_values,y_values,z_values):
        lop.append([x,y,z,lista[3],lista[4],lista[5]])


generate_points_numpy(lista, listb)
lop.append(listb)   

#printing all the path points
for inner_list in lop:
    for element in inner_list:
        print(element, end=" ")
        print()  

print(lista[0],lista[1],listb[0],listb[1],lista[2])
print(len(lop))

#calculating joint angles
# Iterate over each inner list in lop



Pos = []
Rot = []

# Iterate over each inner list in lop
for inner_list in lop:
    # Extract the first three elements for posit
    position_array = np.array(inner_list[:3])
    # Extract the last three elements for rotation
    rotation_array = np.array(inner_list[3:])
    
    # Append position and rotation arrays to Pos and Rot lists respectively
    Pos.append(position_array)
    Rot.append(rotation_array)


joint_angles=[]
flange_rot_new=[]
flange_pos_new=[]

for i in range(len(Rot)):
   rotation_mat = euler_to_rotationzyx(Rot[i])
   poszyx=[Pos[i][2], Pos[i][1], Pos[i][0]]
   T0_7 = get_translational_matrix_mm(rotation_mat,poszyx,libraries)
   T_7_6= T0_7 @ np.linalg.inv(T_6_7)
   posx=T_7_6[0][3]
   posy=T_7_6[1][3]
   posz=T_7_6[2][3]
   eulernew= matrixtozyx(T_7_6[:3,:3])
   flange_rot_new.append(eulernew)
   flange_pos_new.append([posx,posy,posz])
   

for i in range(len(flange_rot_new)):
    x=flange_rot_new[i][0]
    y=flange_rot_new[i][1]
    z=flange_rot_new[i][2]
    pos_ik = np.array([[flange_pos_new[i][0]], [flange_pos_new[i][1]], [flange_pos_new[i][2]]])
    rot_ik=euler_to_rotationzyx([z,y,x])
    temp=obj.InverseKinematics(flange_pos_new[i], rot_ik)
    joint_angles.append(temp)
        
    




#verification code for path points

import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D 

def verify_points_on_line(position_array, lista, listb,flange_pos_new): 
    # Unpack points
     x, y, z = zip(*position_array) 
     # Create 3D scatter plot 
     fig = plt.figure() 
     ax = fig.add_subplot(111, projection='3d') 
     ax.scatter(x, y, z, label='Given Points') 
     # Plot the line between points a and b
     ax.plot([lista[0], listb[0]], [lista[1], listb[1]], [lista[2], listb[2]], 
             color='blue', label='Line from A to B') 
     p = [point[0] for point in flange_pos_new]
     q = [point[1] for point in flange_pos_new]
     r = [point[2] for point in flange_pos_new]

     

    # Plot points
     ax.scatter(p, q, r , color='red', label='Flange')

    
     ax.set_xlabel('X') 
     ax.set_ylabel('Y') 
     ax.set_zlabel('Z') 
     ax.legend() 
     plt.grid(True)
     plt.show() 
     
     
     
verify_points_on_line(Pos, lista, listb,flange_pos_new)

