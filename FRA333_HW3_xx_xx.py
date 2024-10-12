# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1. ชนิตา_6511
2. ฐิติกานต์_6517
3.
'''
#=============================================<คำตอบข้อ 1>======================================================#
#code here
import numpy as np
from HW3_utils import FKHW3  # Assuming FKHW3 is available
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


#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
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


#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
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
    
    # คูณ Jacobian Transpose กับ Wrench (แรง + โ5มเมนต์)
    #ใช้เครื่องหมายลบเพื่อแสดงว่า torque ของข้อต่อเป็นแรงปฏิกิริยาที่ต่อต้านกับแรง (wrench) ที่ปลายมือ
    tau = -np.dot(J_transpose, w)
    
    return tau

#==============================================================================================================#