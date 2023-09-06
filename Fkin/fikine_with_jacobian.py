import numpy as np
from scipy.optimize import minimize

# Define constants for the robot arm (link lengths, initial joint angles, etc.)
link_lengths = [1.0, 1.0, 1.0, 1.0]
initial_joint_angles = [0.0, 0.0, 0.0, 0.0]
target_position = np.array([0.13416297, 1.89188842, 0.        ])  # Target end effector position

# Define the forward kinematics function to compute the end effector position given joint angles
def forward_kinematics(joint_angles):
    transformation_matrix = np.identity(4)
    
    for i in range(len(link_lengths)):
        cos_theta = np.cos(joint_angles[i])
        sin_theta = np.sin(joint_angles[i])
        
        rotation_matrix = np.array([
            [cos_theta, -sin_theta, 0, link_lengths[i]],
            [sin_theta, cos_theta, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        transformation_matrix = np.dot(transformation_matrix, rotation_matrix)
    
    end_effector_position = transformation_matrix[:3, 3]
    return end_effector_position, 

# Define the Jacobian matrix function to compute the sensitivity of end effector position to joint angles
def jacobian_matrix(joint_angles):
    jacobian = np.zeros((3, len(link_lengths)))
    transformation_matrix = np.identity(4)
    
    for i in range(len(link_lengths)):
        cos_theta = np.cos(joint_angles[i])
        sin_theta = np.sin(joint_angles[i])
        
        rotation_matrix = np.array([
            [-sin_theta, -cos_theta, 0, 0],
            [cos_theta, -sin_theta, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0]
        ])
        
        transformation_matrix = np.dot(transformation_matrix, rotation_matrix)
        
        jacobian[:, i] = np.cross(np.array([0, 0, 1]), forward_kinematics(joint_angles) - transformation_matrix[:3, 3])
    
    return jacobian

# Define the error function to calculate the difference between the current and target positions
def error_function(joint_angles):
    current_position = forward_kinematics(joint_angles)
    error = current_position - target_position
    return error

# Define the Levenberg-Marquardt optimization function
def levenberg_marquardt(joint_angles):
    error = error_function(joint_angles)
    jacobian = jacobian_matrix(joint_angles)
    damping_factor = 0.01  # Initial damping factor

    # Levenberg-Marquardt formula
    update_step = np.linalg.solve(jacobian.T @ jacobian + damping_factor * np.eye(4), jacobian.T @ error)
    
    new_joint_angles = joint_angles - update_step
    return new_joint_angles

# Main optimization loop
current_joint_angles = np.array(initial_joint_angles)
max_iterations = 100
tolerance = 1e-6

for iteration in range(max_iterations):
    new_joint_angles = levenberg_marquardt(current_joint_angles)
    error = np.linalg.norm(error_function(new_joint_angles))
    
    if error < tolerance:
        print(f"Converged after {iteration + 1} iterations")
        break
    
    current_joint_angles = new_joint_angles

q = [1.0,1.0, 1.0, 2.0]
print("given angles: ", q)
print("end effector position: ", forward_kinematics(q))

print("Final joint angles:", current_joint_angles)
print("Final end effector position:", forward_kinematics(current_joint_angles))
