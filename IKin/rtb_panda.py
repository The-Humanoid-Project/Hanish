import roboticstoolbox as rtb

# Create a robot model
robot = rtb.models.Panda().ets()

# Instantiate the IK_NR solver
solver = rtb.IK_NR()

# Set the desired end-effector pose
Tep = robot.fkine([0, -0.3, 0, -2.2, 0, 2, 0.7854])

# Solve for the joint coordinates
joint_coords = solver.solve(robot, Tep)
print(Tep)

print("Joint Coordinates:", joint_coords)
