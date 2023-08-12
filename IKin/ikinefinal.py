import roboticstoolbox as rtb
import numpy as np

import arm

Arm = arm.ARM().ets()
solver = rtb.IK_NR(pinv=True)

# # Define the desired end-effector pose
# Tep = Arm.fkine([0, 0.3, 0, -2.2])
# print(Tep)
# print(Tep.shape)

# # Solve for the joint coordinates
# solution = solver.solve(Arm, Tep)
# print("IK solution:\n", solution)

# desired_pos = np.array([[1,0,0,5],
#                         [0,1,0,1],
#                         [0,0,1,0],
# #                         [0,0,0,1]])
# desired_pos = np.array([[  -0.5622,    0.7724,    0.2955,    196.6  ],
#                         [  -0.8085,   -0.5885,    0,        16    ],
#                         [   0.1739,   -0.2389,    0.9553 ,  -55.85 ],
#                         [   0,         0 ,        0 ,       1     ] ])
# print("desired :\n",desired_pos)

# solution = solver.solve(Arm, desired_pos)
# print("IK solution:\n", solution)

# Tep = Arm.fkine(solution)
# print("Tep:\n",Tep)
