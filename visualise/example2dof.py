# import roboticstoolbox as rtb
import matplotlib.pyplot as plt
from roboticstoolbox import DHRobot, RevoluteMDH
import numpy as np


class ARM(DHRobot):

    def __init__(self):

        deg = np.pi/180
        mm = 1e-3
        L = [
            RevoluteMDH(
                a=0.0,
                d=0.0,
                alpha=0.0,
                qlim=np.array([-np.pi/2, np.pi]),
                m=4.970684,
                I=[
                    7.03370e-01,
                    7.06610e-01,
                    9.11700e-03,
                    -1.39000e-04,
                    1.91690e-02,
                    6.77200e-03,
                ],
                G=1,
            ),
            RevoluteMDH(
                a=16.0,
                d=16.0,
                alpha=-np.pi / 2,
                qlim=np.array([-np.pi/2, np.pi/2]),
                m=0.646926,
                I=[
                    7.96200e-03,
                    2.81100e-02,
                    2.59950e-02,
                    -3.92500e-03,
                    7.04000e-04,
                    1.02540e-02,
                ],
                G=1,
            )
        ]
        super().__init__(
            L,
            name="2dof_robot",
        )

robot = ARM()
print(robot)


# q = np.array([0.3, 0.4])  # Joint angles
# robot.q = q
# end_effector_pose = robot.fkine(q)

# fig, ax = plt.subplots()
# robot.plot(q, ax=ax)
# ax.set_xlim([-1, 1])  # Adjust as needed
# ax.set_ylim([-1, 1])  # Adjust as needed
# plt.show()
