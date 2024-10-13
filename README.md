# **Homework 3**
## Question 1

Jacobain Matrix Form

![image](https://github.com/user-attachments/assets/83bba325-68cc-40a3-ae9f-de213c856c43)

Jacobain Matrix of RRR Robot Form

![image](https://github.com/user-attachments/assets/6070514e-029c-4462-9e31-18ede06b3843)


Using the Jacobian Matrix Form and 

- Rotation matrices (R)
- Joint positions (P)
- End effector's rotation matrix (R_e)
- End effector's position (p_e)

from the HW3_utils file to compute the Jacobian Matrix of RRR robot.

### Function

    def endEffectorJacobianHW3(q: list[float]) -> list[float]:

### Parameters

    q (list of float): A list of joint angles [q1,q2,q3][q1​,q2​,q3​] for the 3-DOF robotic arm (in radians).

### Return

    J_e (list of float): A 6x3 Jacobian matrix consisting of:
        Translational Part (J_v): Relating joint velocities to linear velocities of the end effector.
        Rotational Part (J_w): Relating joint velocities to angular velocities of the end effector.


### Code

    def endEffectorJacobianHW3(q:list[float])->list[float]:
    # Get Forward Kinematics values from FKHW3
    R, P, R_e, p_e = FKHW3(q)
    
    # Initialize Jacobian matrices (6x3)
    J_v = np.zeros((3, 3))  # Translational part
    J_w = np.zeros((3, 3))  # Rotational part
    
    # Loop over each joint (there are 3 joints in a 3-DOF manipulator)
    for i in range(3):
        # Get rotation matrix for joint i
        R_i0 = R[:, :, i]
        
        # The joint axis (z-axis for RRR robot) is the third column of R_i0
        z_i = R_i0[:, 2]
        
        # Position of joint i
        p_i = P[:, i]
        
        # Compute the translational part of the Jacobian for joint i
        J_v[:, i] = np.cross(z_i, (p_e - p_i))  # Linear velocity contribution
        
        # The rotational part is just the joint axis for RRR robot
        J_w[:, i] = z_i  # Angular velocity contribution
    
    # Combine translational and rotational parts into a 6x3 Jacobian
    J_e = np.vstack((J_v, J_w))
    
    return J_e
