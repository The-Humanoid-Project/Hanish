import roboticstoolbox as rtb
import numpy as np

import arm

Arm = arm.ARM().ets()
# solver = rtb.IK_NR(pinv=True)
solver = rtb.IK_LM(pinv=True)

## Define the desired end-effector pose
# Tep = Arm.fkine([0, 0.3, 0, -2.2])
# print(Tep)
# print(Tep.shape)

## Solve for the joint coordinates
# solution = solver.solve(Arm, Tep)

desired_pos = np.array([[1,0,0,0],
                        [0,0,1,0],
                        [0,1,0,0.33],
                        [0,0,0,1]])
print("desired :\n",desired_pos)

solution = solver.solve(Arm, desired_pos)
print("IK solution:\n", solution)
