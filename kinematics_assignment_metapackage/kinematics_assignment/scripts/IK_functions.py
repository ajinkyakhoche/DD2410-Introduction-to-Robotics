#! /usr/bin/env python3
import numpy as np
import math
import copy
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
    tolerance = 1e-6
    eps_x = np.ones(6)
    # h = 1e-9
    i = 0

    # x_target = [x, y, z, ?, ?, ?]
    euler_target = find_euler(np.array([R])[0])
    # x_target = [x, y, z, ?, ?, ?]
    x_target = np.hstack((np.array([x, y, z]), euler_target))


    #while True:
    for j in range(5):    
        # Step1.  x_hat = K(theta_hat)     #Forward Kinematics; theta_hat = q
        x_hat, J1 = forward_kinematics(q)    
        
        # Step 2.  eps_x = x_hat - x_target
        eps_x = x_hat - x_target

        #j = j + 1
        
        J_pseudo_inv = np.linalg.pinv(J1)
        # Step 3.  eps_theta = J_pseudo_inv * eps_x
        eps_theta = np.matmul(J_pseudo_inv, eps_x)
        
        # Step 4.  theta_hat = theta_hat - eps_theta    #q = theta_hat
        q = q - eps_theta

    # print('>>>  ITERATION: ' + str(j) + '   <<<')
    # print('MAX ERROR is: ' + str(abs(max(eps_x))))
        
        # if abs(max(eps_x)) < tolerance:
        #     break
    return q


'''x_hat = K(q)'''
def forward_kinematics(q):
    # Design variables
    L = 0.4
    M = 0.39
    d_end_effector = 0.078
    #d_end_effector = 0.01

    alpha = [np.pi/2, -np.pi/2, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2, 0]
    d = [0,0,L,0,M,0,0]
    p = []
    z = []
    T = np.eye(4)
    T[2,3] = 0.311
    p.append(T[0:3, 3])
    z.append(T[0:3, 2])
    T0 = np.eye(4)
    T0[2,3] = d_end_effector
    for i in range(len(q)):
        temp = np.array([[np.cos(q[i]), -np.sin(q[i])*np.cos(alpha[i]), np.sin(q[i])*np.sin(alpha[i]), 0],
                    [np.sin(q[i]), np.cos(q[i])*np.cos(alpha[i]), -np.cos(q[i])*np.sin(alpha[i]), 0],
                    [0, np.sin(alpha[i]), np.cos(alpha[i]), d[i]],
                    [0, 0, 0, 1]])
        T = np.dot(T,temp)
        p.append(T[0:3, 3])
        z.append(T[0:3, 2])

    T = np.dot(T, T0)
    p.append(T[0:3, 3])
    z.append(T[0:3, 2])

    T_70 = T

    #x_hat = np.hstack((np.array([T_70[0][3], T_70[1][3], T_70[2][3]]), (np.reshape(T_70[0:3, 0:3], (1,-1))).ravel()))
    euler_hat = find_euler(T_70[0:3, 0:3])
    x_hat = np.hstack((np.array([T_70[0][3], T_70[1][3], T_70[2][3]]), euler_hat))
    
    # calculate jacobian
    J = []
    for i in range(len(q)):
        J_temp = np.cross(z[i], (p[-1] - p[i]))
        J.append(np.hstack((J_temp, z[i])))
    J_arr = np.array(J)
    return x_hat, J_arr.T


def isclose(x, y, rtol=1.e-5, atol=1.e-8):
    return abs(x-y) <= atol + rtol * abs(y)

def find_euler(R):
    '''
    psi, theta and phi are euler angles along x,y,z
    pseudo code courtesy:

        Computing euler angles from rotation matrix (page 5 of 7)
                                - Gregory G Slabaugh
    ''' 
    phi = 0.0
    if isclose(R[2,0],-1.0):
        theta = math.pi/2.0
        psi = math.atan2(R[0,1],R[0,2])
    elif isclose(R[2,0],1.0):
        theta = -math.pi/2.0
        psi = math.atan2(-R[0,1],-R[0,2])
    else:
        theta = -math.asin(R[2,0])
        cos_theta = math.cos(theta)
        psi = math.atan2(R[2,1]/cos_theta, R[2,2]/cos_theta)
        phi = math.atan2(R[1,0]/cos_theta, R[0,0]/cos_theta)
    return np.array([psi, theta, phi])

kuka_IK([-0.217,  0.,     0.84 ], [[0, 0, -1], [0, 1, 0], [1, 0, 0]], [0, 1.12, 0, 1.71, 0, 1.84, 0])
