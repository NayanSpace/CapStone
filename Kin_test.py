import numpy as np
from UR_Kinematics import inverse_kinematic_solution, DH_matrix_UR5e

transform = np.matrix([[0.00000000e+00, 0.00000000e+00, -1.00000000e+00, 3.00000000e-01],
                       [0.00000000e+00, 1.00000000e+00, 0.00000000e+00, 3.00000000e-01],
                       [1.00000000e+00, 0.00000000e+00, 0.00000000e+00, 3.00000000e-01],
                       [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
                        
IKS = inverse_kinematic_solution(DH_matrix_UR5e, transform)
#print(IKS)              # all solutions

for i in range(8):
    print(i)
    print("[" + str(IKS[0, i]) + ", " + str(IKS[1, i]) + ", " + str(IKS[2, i]) + ", " + str(IKS[3, i]) + ", " + str(IKS[4, i]) + ", " + str(IKS[5, i]) + "]")
