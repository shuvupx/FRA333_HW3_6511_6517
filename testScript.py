# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ex: ธนวัฒน์_6461)
1. ชนิตา_6511
2. ฐิติกานต์_6517
'''

#ใส่ค่า q&w
q_input = [0,0,0]
w_input = [0,10,0,5,5,0]
print("q_input = ", q_input)
print("q_input = ", w_input)

import roboticstoolbox as rtb
from math import pi
from spatialmath import SE3
import numpy as np

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


#===========================================<ตรวจคำตอบข้อ 1>====================================================#
#code here
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




#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here
from FRA333_HW3_6511_6517 import checkSingularityHW3
def Check_checkSingularityHW3(q:list[float])->bool:
    """
    ฟังก์ชันนี้ใช้ตรวจสอบว่าแขนกลอยู่ในสภาวะ Singularity หรือไม่
    โดยการตรวจสอบ determinant ของ Jacobian ที่ลดรูปแล้ว
    """
    epsilon = 0.001
    # คำนวณ Jacobian matrix จาก q
    J_check = Check_endEffectorJacobianHW3(q)

    # ตัด 3 แถวล่างออก
    J_check_reduced = J_check[:3, :]
    
    # คำนวณ determinant ของ Jacobian
    det_J_check_reduced = np.linalg.det(J_check_reduced)
    
    # ตรวจสอบเงื่อนไข Singularity
    if abs(det_J_check_reduced) < epsilon:
        return 1  # อยู่ในสภาวะ Singularity
    else:
        return 0  # ไม่อยู่ในสภาวะ Singularity

print('\n')
print('Check Q2')
if (checkSingularityHW3(q_input) != Check_checkSingularityHW3(q_input)):
    print("checkSingularity Not Correct")

else:
    print("checkSingularity Correct")

#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
#code here
from FRA333_HW3_6511_6517 import computeEffortHW3
def Check_computeEffortHW3(q: list[float], w: list[float]) -> np.ndarray:
    # คำนวณ Jacobian Matrix จาก q
    J_check = Check_endEffectorJacobianHW3(q)
    
    # นำ Transpose ของ Jacobian มาคำนวณ
    J_check_transpose = np.transpose(J_check)
    
    # คูณ Jacobian Transpose กับ Wrench (แรง + โมเมนต์)
    # ใช้เครื่องหมายลบเพื่อแสดงว่า torque ของข้อต่อเป็นแรงปฏิกิริยาที่ต่อต้านกับแรง (wrench) ที่ปลายมือ
    tau_check = -np.dot(J_check_transpose, w)
    
    return tau_check

# Get the results from both functions
effort1 = computeEffortHW3(q_input, w_input)
effort2 = Check_computeEffortHW3(q_input, w_input)

# Calculate the acceptable error margin (0.001% of the maximum value in effort1)
tolerance = 0.001 / 100  # 0.001%


print('\n')
print('Check Q3')
print("Effort from computeEffortHW3:\n", effort1)
print("Effort from Check_computeEffortHW3:\n", effort2)
# Check if the efforts are approximately equal within the specified tolerance
if not np.allclose(effort1, effort2, atol=tolerance * np.max(np.abs(effort1))):
    print("Effort Not Correct")
else:
    print("Effort Correct")

# Print both efforts for comparison

#==============================================================================================================#