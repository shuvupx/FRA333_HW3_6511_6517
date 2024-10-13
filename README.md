# **Homework 3**

นักศึกษาสามารถประยุกต์ความรู้ในเรื่องของการหาจลศาสตร์เชิงอนุพันธ์เพื่อใช้ในการควบคุมหุ่นยนต์ RRR ที่มีลักษณะโครงสร้างดังรูปที่ 1 โดยใช้สมการ Forward Kinematics ที่ทางผู้ผลิตให้มาในฟังก์ชั่น FKHW3 ใน HW3_utils.py

![image](https://github.com/user-attachments/assets/ae1933ae-f974-44ea-8029-00ab40b34c9a)

# Table of Contents
- [Part 1 Solve Problems ](#Part-1-Solve-Problems)
  - [Question 1](#Question-1)
  - [Question 2](#Question-2)
  - [Question 3](#Question-3)
- [Part 2 Check the Answer](#Part-2-Check-the-Answer)
    - [Question 1](#Check-Question-1)
    - [Question 2](#Check-Question-2)
    - [Question 3](#Check-Question-3)
  
# Part 1 Solve Problems
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

**Function**

    def endEffectorJacobianHW3(q: list[float]) -> list[float]:

**Parameters**

    q (list of float): A list of joint angles [q1,q2,q3][q1​,q2​,q3​] for the 3-DOF robotic arm (in radians).

**Return**

    J_e (list of float): A 6x3 Jacobian matrix consisting of:
        Translational Part (J_v): Relating joint velocities to linear velocities of the end effector.
        Rotational Part (J_w): Relating joint velocities to angular velocities of the end effector.


**Code**

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


## Question 2

The function computes the determinant of the top three rows of the Jacobian matrix. The condition for singularity can be mathematically expressed as:

**∣∣ det(J∗(q))∣∣ < ϵ**

Where:

    J∗(q) is the reduced Jacobian matrix obtained from the full Jacobian.
    ϵ is a small threshold value (0.001) to account for numerical inaccuracies.

If the absolute value of the determinant is less than this threshold, the arm is considered to be in a singularity state.

**Function**

    def checkSingularityHW3(q: list[float]) -> bool:

**Parameters**

    q (list of float): A list of joint angles [q1,q2,q3][q1​,q2​,q3​] (in radians) for the 3-DOF robotic arm.

**Returns**

    bool: Returns 1 if the robotic arm is in a singularity state; otherwise, returns 0.

**Code**

    def checkSingularityHW3(q:list[float])->bool:
    """
    ฟังก์ชันนี้ใช้ตรวจสอบว่าแขนกลอยู่ในสภาวะ Singularity หรือไม่
    โดยการตรวจสอบ determinant ของ Jacobian ที่ลดรูปแล้ว
    """
    epsilon = 0.001
    # คำนวณ Jacobian matrix จาก q
    J = endEffectorJacobianHW3(q)

    # ตัด 3 แถวล่างออก
    J_reduced = J[:3, :]
    
    # คำนวณ determinant ของ Jacobian
    det_J_reduced = np.linalg.det(J_reduced)
    
    # ตรวจสอบเงื่อนไข Singularity
    if abs(det_J_reduced) < epsilon:
        return 1  # อยู่ในสภาวะ Singularity
    else:
        return 0  # ไม่อยู่ในสภาวะ Singularity

## Question 3

The torques at the joints are calculated using the following equation:

![image-14](https://github.com/user-attachments/assets/3aa6f8e3-a06e-42e3-8d19-b1773121a17c)

Where:

    JT is the transpose of the Jacobian matrix.
    w is the wrench vector applied at the end effector.


**Function**

    def computeEffortHW3(q: list[float], w: list[float]) -> list[float]:
    # Description and code implementation below

**Parameters**

    q (list of float): A vector representing the joint configuration of the robotic arm (3-dimensional).
    w (list of float): A vector representing the wrench (forces and moments) applied at the end-effector (6-dimensional). 
    This includes:
        w[0],w[1],w[2]: Force components (Fx,Fy,Fz)
        w[3],w[4],w[5]: Moment components (Mx,My,Mz)
**Return**

    tau (numpy.ndarray): A vector of torques for each joint (3-dimensional).



**Code**

    def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    """
    คำนวณค่า Effort (Torque) ที่กระทำกับแต่ละข้อต่อของหุ่นยนต์ RRR 
    เมื่อมีแรงและโมเมนต์มากระทำที่ตำแหน่งปลายมือ
    
    Parameters:
    q : list[float] 
        เวกเตอร์ของ Joint Configuration (3 มิติ)
    w : list[float] 
        เวกเตอร์แรงและโมเมนต์ (6 มิติ) ที่มากระทำกับปลายมือ
    
    Returns:
    tau : np.ndarray 
        เวกเตอร์ Torque ของแต่ละข้อต่อ (3 มิติ)
    """
    # คำนวณ Jacobian Matrix จาก q
    J = endEffectorJacobianHW3(q)
    
    # นำ Transpose ของ Jacobian มาคำนวณ
    J_transpose = np.transpose(J)
    
    # คูณ Jacobian Transpose กับ Wrench (แรง + โมเมนต์)
    #ใช้เครื่องหมายลบเพื่อแสดงว่า torque ของข้อต่อเป็นแรงปฏิกิริยาที่ต่อต้านกับแรง (wrench) ที่ปลายมือ
    tau = -np.dot(J_transpose, w)
    
    return tau


# Part 2 Check the Answer

Input Values:

    - q_input is a list representing the joint angles of the robotic arm in radians. In this case, all joints are initialized to 0.
  
    - w_input is a list representing the wrench applied at the end effector, defined as six components: force and moment (fx, fy, fz, mx, my, mz).

We fixed it to:

    q_input = [0,0,0]
    w_input = [0,10,0,5,5,0]


**Define Robot with MDH Parameters**

![image](https://github.com/user-attachments/assets/d0aef5bb-b10f-4f96-92c2-95a075ebbe05)

**Code**

    # Define the robot using Modified Denavit-Hartenberg parameters
    robot = rtb.DHRobot(
        [
            rtb.RevoluteMDH(d=0.0892, offset=pi),        # Joint 1
            rtb.RevoluteMDH(alpha=pi/2, offset=0),       # Joint 2
            rtb.RevoluteMDH(a=-0.425, offset=0),         # Joint 3
            
        ],
        tool = SE3([
            [np.cos(-pi/2), 0, np.sin(-pi/2), -0.47443],
            [0, 1, 0, -0.093],
            [-np.sin(-pi/2), 0, np.cos(-pi/2), 0.109],
            [0, 0, 0, 1]  # Homogeneous coordinate
        ]),
        name="RRR_Robot"
    )
    
    # Print the robot model to verify
    print(robot)

**Output**

![Screenshot from 2024-10-13 16-20-04](https://github.com/user-attachments/assets/520d298b-9c15-4bec-8c7b-98b996e9efe1)

    
## Check Question 1

Check whether the Jacobian matrix produced by the user-defined function is consistent with the one obtained from Robotics Toolbox.

**Function**

    Check_endEffectorJacobianHW3(q: list[float]) -> np.ndarray:

**Input**

    The joint configuration is specified using q_input, which represents the angles of each joint in the robotic arm.

**Return**

    returns the Jacobian matrix calculated using the Robotics Toolbox.


**Code**

    from FRA333_HW3_6511_6517 import endEffectorJacobianHW3
    def Check_endEffectorJacobianHW3(q: list[float]) -> np.ndarray:
        J_check = robot.jacob0(q)
        return J_check
    
    # Get the results from both functions
    jacobian1 = endEffectorJacobianHW3(q_input)
    jacobian2 = Check_endEffectorJacobianHW3(q_input)
    
    # Calculate the acceptable error margin (0.001% of the maximum value in jacobian1)
    tolerance = 0.001 / 100  # 0.001%
    
    print('\n')
    print('Check Q1')
    # Print both Jacobians for comparison
    print("Jacobian from endEffectorJacobianHW3:\n", jacobian1)
    print("Jacobian from Check_endEffectorJacobianHW3:\n", jacobian2)
    # Check if the Jacobians are approximately equal within the specified tolerance
    if not np.allclose(jacobian1, jacobian2, atol=tolerance * np.max(np.abs(jacobian1))):
        print("Jacobian Matrix Not Correct")
    else:
        print("Jacobian Matrix Correct")
        
**Comparison Process**

    1. The Jacobian matrix is calculated using both the custom function and the Robotics Toolbox.
    2. An acceptable error margin is defined (0.001% of the maximum value in jacobian1).
    3. Both Jacobians are printed for visual comparison.
    4. The np.allclose() function is used to check if the two matrices are approximately equal within the defined tolerance.
    
**Output**

![Screenshot from 2024-10-13 16-25-15](https://github.com/user-attachments/assets/6cbfdb8f-f8ab-4e10-95b0-5d6624232daa)



## Check Question 2

## Check Question 3











    
