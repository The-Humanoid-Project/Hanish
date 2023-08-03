import numpy as np

mm = 10**-3

# Change these thetas 
theta1 = 0
theta2 = 0
theta3 = 0
theta4 = 0

# DH Parameters
a =     [0,     16,     60,         129] 
alpha = [0,     -np.pi/2,np.pi/2,   0  ] 
d =     [0,     16,     0,          0  ] 
theta = [theta1 ,theta2 ,theta3 ,theta4] 

T = []
for i in range(len(theta)):
    Ti = np.array([
        [np.cos(theta[i]), -np.sin(theta[i])*np.cos(alpha[i]), np.sin(theta[i])*np.sin(alpha[i]), a[i]*np.cos(theta[i])],
        [np.sin(theta[i]), np.cos(theta[i])*np.cos(alpha[i]), -np.cos(theta[i])*np.sin(alpha[i]), a[i]*np.sin(theta[i])],
        [0, np.sin(alpha[i]), np.cos(alpha[i]), d[i]],
        [0, 0, 0, 1]
    ])
    T.append(Ti)

T_total = T[0] @ T[1] @ T[2] @ T[3]

# print (T_total)

row , col = T_total.shape
for i in range(row):
    for j in range(col):
        if abs(T_total[i][j]) < 0.001:
            T_total[i][j] = 0
        
    

position = T_total[:3, 3]
orientation = T_total[:3, :3]



print("End effector position:", position)
print("End effector orientation:\n", orientation)
