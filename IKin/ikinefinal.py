import roboticstoolbox as rtb
import numpy as np
from spatialmath.pose3d import SE3
# from time import sleep

import arm
Arm = arm.ARM().ets()
solver = rtb.IK_NR(pinv=True)

# Define the desired end-effector pose
Tep = Arm.fkine([np.pi/2, np.pi/2, np.pi/2, np.pi/2])
# print("Tep:\n",Tep)

# Solve for the joint coordinates
solution = solver.solve(Arm, Tep)
print("IK solution:\n", solution)

print("****************************")

desired_pos = np.array([[0.0,   1.0,    0.0,   -145.0],
                        [0.0,   0.0,    1.0,    16.0],
                        [1.0,   0.0,    0.0,   -60.0],
                        [0.0,   0.0,    0.0,    1.0]])
# desired_pos = SE3(desired_pos)
# print("desired :\n",desired_pos)

# solution = solver.solve(Arm, desired_pos)
# print("IK solution:\n", solution)

#plot
Arm.plot(np.zeros(4), block=True)

