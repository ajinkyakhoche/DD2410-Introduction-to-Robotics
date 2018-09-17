#! /usr/bin/env python3
import numpy as np
"""
    # {student full name} Ajinkya Khoche
    # {student email} khoche@kth.se
"""
# Design Variables of robot
l0 = 0.07
l1 = 0.3 
l2 =  0.35

def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]

    """ 
    Fill in your IK solution here and return the three joint values in q

    Let phi1, phi2 be rotation1 and 2 respectively
    Let 't' be the translation
    Solution: Page 110 of Craig
    """
    t = z
    # phi1 = np.arcsin(((x - l0)**2 + y**2 + l1**2 - l2**2) / (2 * np.sqrt(2) * l1)) - np.pi / 4
    # phi2 = np.arcsin((y - l1 * np.sin(phi1)/l2)) - phi1
    c2 = ((x - l0)**2 + y**2 - l1**2 - l2**2) / (2*l1*l2)
    s2 = np.sqrt(1 - c2**2)
    phi2 = np.arctan2(s2,c2)

    # calculate k1, k2
    k1 = l1 + l2 * c2
    k2 = l2 * s2
    phi1 = np.arctan2(y,(x-l0)) - np.arctan2(k2,k1)
    q = [phi1, phi2, t]
    return  q

def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements

    # print('point is: ' + str(point))
    # print('R is: ' + str(R))
    # print('q is: ' + str(q))

    """
    Fill in your IK solution here and return the seven joint values in q
    x7 = T_76*T_65*T_54*T_43*T_32*T_21*T_10 * x0, where x7 is position of end effector
    
    Steps involved: slide 26 of Lecture 4: differntial kinematics
    Step1.  x_hat = K(theta_hat)     #Forward Kinematics; theta_hat = q
        x_target = [x, y, z, ?, ?, ?]
    2.  eps_x = x_hat - x_target
    3.  eps_theta = J_pseudo_inv * eps_x
    4.  theta_hat = theta_hat - eps_theta     
    """
    # Design variables
    L = 0.4
    M = 0.39
    d_end_effector = 0.078
    tolerance = 0.1
    eps_x = np.ones(6)
    i = 0

    while np.linalg.norm(eps_x) > tolerance:
        T_01 = find_transformation(np.pi/2, 0, 0, q[0])
        T_12 = find_transformation(-np.pi/2, 0, 0, q[1])
        T_23 = find_transformation(-np.pi/2, L, 0, q[2])
        T_34 = find_transformation(np.pi/2, 0, 0, q[3])
        T_45 = find_transformation(np.pi/2, M, 0, q[4])
        T_56 = find_transformation(-np.pi/2, 0, 0, q[5])
        T_67 = find_transformation(0, d_end_effector, 0, q[6])

        T_07 = np.linalg.multi_dot([T_01, T_12, T_23, T_34, T_45, T_56, T_67]) 
        T_70 = np.linalg.inv(T_07)

        x_0 = np.array([0,0,0,1])

        # Step1.  x_hat = K(theta_hat)     #Forward Kinematics; theta_hat = q
        # x_target = [x, y, z, ?, ?, ?]
        euler_hat = find_euler(T_70[0:3, 0:3])
        #x_hat = np.array([T_70[0][3], T_70[1][3], T_70[2][3], np.arccos(T_70[0][2]), np.arccos(T_70[1][2]), np.arccos(T_70[2][2])])
        x_hat = np.hstack((np.array([T_70[0][3], T_70[1][3], T_70[2][3]]), euler_hat))
        
        #x_hat = np.hstack((np.array([T_70[0][3], T_70[1][3], T_70[2][3]]), T_70[0,0:3], T_70[1,0:3], T_70[2,0:3] ))

        euler_target = find_euler(np.array([R])[0])
        #x_target = np.array([x, y, z, np.arccos(R[2][2]), np.arccos(R[2][2]), np.arccos(R[0][0])]) ### NOTE: this might be wrong
        x_target = np.hstack((np.array([x, y, z]), euler_target))
        #x_target = np.hstack((np.array([x, y, z]), R[0], R[1], R[2]))
        # Step 2.  eps_x = x_hat - x_target
        eps_x = x_hat - x_target

        J = calc_jacobian(T_01, T_12, T_23, T_34, T_45, T_56, T_67, T_07)

        J_pseudo_inv = np.dot(J.T, np.linalg.inv(np.dot(J, J.T)))
        # Step 3.  eps_theta = J_pseudo_inv * eps_x
        eps_theta = np.dot(J_pseudo_inv, eps_x)
        # Step 4.  theta_hat = theta_hat - eps_theta    #q = theta_hat
        q = q - eps_theta

        i = i + 1
        print('>>>  ITERATION: ' + str(i) + '   <<<')
        print('euler_target =' + str(euler_target))
        print('euler_hat = ' + str(euler_hat))
        print('Magnitude is: ' + str(np.linalg.norm(eps_x)))
        print('----------------------------------------')
    return q


'''
uses DH parameters to find Transformation matrix
T_i-1,i = Trans(z_i-1 , d_i) * Rot(z_i-1 , theta_i) * Trans(x_i , a_i) * Rot(x_i , alpha_i) 
'''
def find_transformation(alpha_i, d_i, a_i, theta_i):
    # T1 = trans(2,d_i)
    # R1 = rot(2,theta_i)
    # T2 = trans(0,a_i)
    # R2 = rot(0,alpha_i)

    # T_ji = T1 * R1 * T2 * R2    # T_i-1,i
    T_ji = np.eye(4)

    T_ji[0,0] = np.cos(theta_i)
    T_ji[0,1] = -1 * np.sin(theta_i) * np.cos(alpha_i)
    T_ji[0,2] = np.sin(theta_i) * np.sin(alpha_i)
    T_ji[0,3] = a_i * np.cos(theta_i)

    T_ji[1,0] = np.sin(theta_i)
    T_ji[1,1] = np.cos(theta_i) * np.cos(alpha_i)
    T_ji[1,2] = -1 * np.cos(theta_i) * np.sin(alpha_i)
    T_ji[1,3] = a_i * np.sin(theta_i) 

    T_ji[2,1] = np.sin(alpha_i)
    T_ji[2,2] = np.cos(alpha_i)
    T_ji[2,3] = d_i

    return T_ji # returns T_i-1,i


'''
calculate jacobian 
page 112 of Springer Robotics book
'''
def calc_jacobian(T_01, T_12, T_23, T_34, T_45, T_56, T_67, T_07):
    # calculate J_O1
    z0 = np.array([0,0,1])
    p0 = np.array([0,0,0])  # first three elements of [0,0,0,1]
    p7 = extract_p(T_07)
    J1 = np.cross(z0, (p7 - p0))
    J1 = np.hstack((J1, z0))

    # calculate J_O2
    z1 = extract_z(T_01)
    p1 = extract_p(T_01)
    J2 = np.hstack((np.cross(z1,(p7 - p1)), z1))

    # calculate J_O3
    z2 = extract_z(np.linalg.multi_dot([T_01, T_12]))
    p2 = extract_p(np.linalg.multi_dot([T_01, T_12]))
    J3 = np.hstack((np.cross(z2,(p7 - p2)), z2))

    # calculate J_O4
    z3 = extract_z(np.linalg.multi_dot([T_01, T_12, T_23]))
    p3 = extract_p(np.linalg.multi_dot([T_01, T_12, T_23]))
    J4 = np.hstack((np.cross(z3,(p7 - p3)), z3))

    # calculate J_O5
    z4 = extract_z(np.linalg.multi_dot([T_01, T_12, T_23, T_34]))
    p4 = extract_p(np.linalg.multi_dot([T_01, T_12, T_23, T_34]))
    J5 = np.hstack((np.cross(z4,(p7 - p4)), z4))

    # calculate J_O6
    z5 = extract_z(np.linalg.multi_dot([T_01, T_12, T_23, T_34, T_45]))
    p5 = extract_p(np.linalg.multi_dot([T_01, T_12, T_23, T_34, T_45]))
    J6 = np.hstack((np.cross(z5,(p7 - p5)), z5))
    
    # calculate J_O7
    z6 = extract_z(np.linalg.multi_dot([T_01, T_12, T_23, T_34, T_45, T_56]))
    p6 = extract_p(np.linalg.multi_dot([T_01, T_12, T_23, T_34, T_45, T_56]))
    J7 = np.hstack((np.cross(z6,(p7 - p6)), z6))

    J = np.vstack((J1, J2, J3, J4, J5, J6, J7))

    return J.T

def extract_p(T):
    return T[0:3, 3]

def extract_z(T):
    return T[0:3, 2]

def find_euler(R):
    '''
    psi, theta and phi are euler angles along x,y,z
    pseudo code courtesy:

        Computing euler angles from rotation matrix (page 5 of 7)
                                - Gregory G Slabaugh
    ''' 
    if R[2,0] > 1:
        R[2,0] = round(R[2,0])  
    if R[2,0] == 1 or R[2,0] == -1:
        phi = 0

        if(R[2,0] == -1):
            theta = np.pi/2
            psi = phi + np.arctan2(R[0,1], R[0,2])
        else:
            theta = -np.pi/2
            psi = -phi + np.arctan2(-R[0,1], -R[0,2])
    else:
        theta = - np.arcsin(R[2,0])
        if abs(R[2, 1]/np.cos(theta)) < 1e-4 and abs(R[2, 2]/np.cos(theta)) < 1e-4:
            psi = 0
        else:
            psi = np.arctan2(R[2, 1]/np.cos(theta), R[2, 2]/np.cos(theta))
        
        if abs(R[1, 0]/np.cos(theta)) < 1e-4 and abs(R[0, 0]/np.cos(theta)) < 1e-4:
            phi = 0
        else:
            phi = np.arctan2(R[1, 0]/np.cos(theta), R[0, 0]/np.cos(theta))
    
    if abs(abs(psi) - np.pi) < 1e-5 :
        psi = 0
    if abs(abs(phi) - np.pi) < 1e-5:
        phi = 0

    return np.array([psi, theta, phi])


kuka_IK([-0.217,  0.,     0.84 ], [[0, 0, -1], [0, 1, 0], [1, 0, 0]], [0, 1.12, 0, 1.71, 0, 1.84, 0])