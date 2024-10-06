# Created by Indraneel on 14/09/2024
import math
import numpy as np 
from .kinematics import Kinematics
import control

class OptimalControl:
    """
    The class consists of functions to implement a tracking
    controller for a planar RR manipulator with contact forces
    The goal is to track a certain position and velocity inspite of
    the dynamics of the system

    1. State of the system x is [q1, q2, q1dot, q2dot]
    2. Input u is [T1, T2] which are joint torques 
    3. We first implement xdot = f(x,u) 
    4. Then we linearise the above dynamics about an operating point of the manipulator
    to get state space form xdot = Ax + Bu 
    5. Once we have A and B matrices we get the feedback gain matrix K using optimal 
    control theory to get du = -Kx
    6. Then finally our control input will be u = uref -Kx
    """
    def __init__(self,robot_parameters, problem_parameters, kinematics):
        self.robot_parameters = robot_parameters
        self.problem_parameters = problem_parameters
        self.kinematics = kinematics



    def f(self,x,u):
        """
        Nonlinear dynamics of the system

        Parameters:
        _____________
        x:  np.ndarray 4x1
        u: np.ndarray 2x1

        Returns:
        __________
        xdot: np.ndarray 4x1
        """
        q1 = x[0].item()
        q2 = x[1].item()
        q1dot = x[2].item()
        q2dot = x[3].item()
        qdot = np.array([[q1dot],[q2dot]])
        
        l1 = self.robot_parameters['l1']
        l2 = self.robot_parameters['l2']
        m1 = self.robot_parameters['m1']
        m2 = self.robot_parameters['m2']

        Mq = np.array([[((m1+m2)*l1*l1)+(m2*l2*l2)+(2*m2*l1*l2*math.cos(q2)), (m2*l2*l2)+(m2*l1*l2*math.cos(q2))],
                       [(m2*l2*l2)+(m2*l1*l2*math.cos(q2)),             (m2*l2*l2)]])
        Mq_inv = np.linalg.inv(Mq)

        Cq = np.array([[0.0, -m2*l1*l2*(2*q1dot+q2dot)*math.sin(q2)],
                       [0.5*m2*l1*l2*(2*q1dot+q2dot)*math.sin(q2), -0.5*m2*l1*l2*q1dot*math.sin(q2)]])
        
        B = np.eye(2)

        qddot = Mq_inv @ (B@u - Cq@qdot)
        xdot = np.array([[q1dot],[q2dot],[qddot[0].item()],[qddot[1].item()]])

        return xdot
    
    def linearize_numerical_perturbation(self, x, u):
        """
        Linearises dynamics about an operating point
        
        Parameters:
        _____________
        x:  np.ndarray 4x1
        u: np.ndarray 2x1

        Returns:
        __________
        A: np.ndarray 4x4
        B: np.ndarray 4x2
        """
        n = 4 
        m = 2
        A_lin = np.zeros((n,n))
        B_lin = np.zeros((n,m))

        xdot = self.f(x,u)
        pert = 1e-2

        # A matrix
        for i in range(0,n):
            x_pert = np.copy(x)
            x_pert[i] = x_pert[i] + pert
            xdot_pert = self.f(x_pert,u)
            A_lin[:,i] = ((xdot_pert-xdot)/pert).flatten()

        # B matrix
        for j in range(0,m):
            u_pert = np.copy(u)
            u_pert[j] = u_pert[j] + pert
            xdot_pert = self.f(x,u_pert)
            B_lin[:,j] = ((xdot_pert-xdot)/pert).flatten()

        return A_lin, B_lin

    def linearize(self, x, u):
        """
        Linearises dynamics about an operating point
        
        Parameters:
        _____________
        x:  np.ndarray 4x1
        u: np.ndarray 2x1

        Returns:
        __________
        A: np.ndarray 4x4
        B: np.ndarray 4x2
        """
        n = 4 
        m = 1
        A_lin = np.zeros((n,n))
        B_lin = np.zeros((n,m))

        q1 = x[0].item()
        q2 = x[1].item()
        q1dot = x[2].item()
        q2dot = x[3].item()
        qdot = np.array([[q1dot],[q2dot]])
        
        l1 = self.robot_parameters['l1']
        l2 = self.robot_parameters['l2']
        m1 = self.robot_parameters['m1']
        m2 = self.robot_parameters['m2']

        M11 = ((m1+m2)*l1*l1)+(m2*l2*l2)+(2*m2*l1*l2*math.cos(q2))
        M12 =  (m2*l2*l2)+(m2*l1*l2*math.cos(q2))
        M21 = (m2*l2*l2)+(m2*l1*l2*math.cos(q2))
        M22 =   (m2*l2*l2)
        Mq = np.array([[M11,M12],
                       [M21,M22]])
        Mq_inv = np.linalg.inv(Mq)
        # print(Mq_inv.shape)

        desired_force = self.problem_parameters['desired_force_on_the_wall_N']
        F = np.array([[desired_force],[0.0]])

        dJdq11 = -l1*math.cos(q1)-l2*math.cos(q1+q2)
        dJdq12 = -l2*math.cos(q1+q2)
        dJdq21 = -l1*math.sin(q1)-l2*math.sin(q1+q2)
        dJdq22 = -l2*math.sin(q1+q2)
        dJdq = np.array([[dJdq11*F[0][0], dJdq12*F[1][0]],
                        [dJdq21*F[0][0], dJdq22*F[1][0]]])
        
        dCdq11 = 0.0
        dCdq12 = -m2*l1*l2*(2*q1dot+q2dot)*math.cos(q2)
        dCdq21 = 0.0
        dCdq22 = -0.5*m2*l1*l2*q1dot*math.cos(q2)
        dCdq = np.array([[dCdq11*q1dot, dCdq12*q2dot],
                        [dCdq21*q1dot,dCdq22*q2dot]])
        
        dCdqdot11 = 0.0
        dCdqdot12 = -m2*l1*l2*math.sin(q2)
        dCdqdot21 = m2*l1*l2*math.sin(q2)
        dCdqdot22 = 0.0
        dCdqdot = np.array([[dCdqdot11*q1dot, dCdqdot12*q2dot],
                            [dCdqdot21*q1dot, dCdqdot22*q2dot]])
        

        A_lin = np.block([
            [np.zeros((2,2)), np.identity(2)],
            [Mq_inv @ (dJdq - dCdq ), Mq_inv @(-dCdqdot) ]
        ])

        B = np.eye(2)
        B_lin = np.block([
            [np.zeros((2,2))],
            [Mq_inv @ B]
        ])

        return A_lin,B_lin
    
    def feedforward_term(self,x):
        """
        Parameters
        _________________
        x:  np.ndarray 4x1

        Returns 
        _________________
        ff: np.ndarray 2x1
        
        """
        q1 = x[0].item()
        q2 = x[1].item()
    
        desired_force = self.problem_parameters['desired_force_on_the_wall_N']
        F = np.array([[desired_force],[0.0]])
        jacobian = self.kinematics.jacobian(np.array([[q1],[q2]]))
        jacobian_T = jacobian.T
        
        ff = jacobian_T@F
        return ff

    
    def tracking_input(self, A, B, K, C, ja):
        """
            Addition of a tracking input allows us to shift the equilibrium point of the system 
            (where the state will converge to) towards our desired state,
            this function calculates and returns the tracking input

            State
            j1
            j2
            w1
            w2

            Parameters:
            _____________
            A:  np.ndarray 4x4
            B: np.ndarray 4x2
            K:  np.ndarray 2x4
            C: np.ndarray 4x4
            ja: np.ndarray 2x1

            Returns:
            __________
            v: np.ndarray 4x1
        """
        
        # Calculate ydes
        ydes = np.zeros((4,1))

        # Maintain end affector coordinates along the wall
        end_affector_coordinates = self.kinematics.forward_kinematics(ja)

        # print(end_affector_coordinates)
        desired_end_affector_coordinates = np.copy(end_affector_coordinates)
        desired_end_affector_coordinates[0] += 0.15
        desired_end_affector_coordinates[0] = min(desired_end_affector_coordinates[0],self.problem_parameters['distance_from_base_to_wall_m'])
        # desired_end_affector_coordinates[1] -= 0.01

        ydes_ja = self.kinematics.inverse_kinematics(desired_end_affector_coordinates)

        # Linear velocity along the wall
        desired_velocity_vec =  np.array([[0.0],[-self.problem_parameters['desired_velocity_along_the_wall_ms']]])
        jacobian = self.kinematics.jacobian(ja)
        jacobian_inv = np.linalg.pinv(jacobian)
        ydes_w = jacobian_inv @ desired_velocity_vec


        # Desired state!
        ydes[0][0] = ydes_ja[0][0]
        ydes[1][0] = ydes_ja[1][0]
        ydes[2][0] = ydes_w[0][0]
        ydes[3][0] = ydes_w[1][0]

        return ydes


if __name__ == "__main__":
    robot_parameters = {
        'l1': 1.0,
        'l2': 1.0,
        'm1': 1.0,
        'm2': 1.0
    }
    kinematics = Kinematics(robot_parameters)
    problem_parameters = {
        'distance_from_base_to_wall_m' : 1.5, #in m
        'desired_force_on_the_wall_N' : 0.5, #in N
        'desired_velocity_along_the_wall_ms' : 0.1, #in m/s
    }
    controller = OptimalControl(robot_parameters, problem_parameters,kinematics)

    ja = np.array([[0.78],
                   [-1.44]])
    print(ja.shape)
    x = np.array([[ja[0][0]], [ja[1][0]],[0.0],[-0.1]])
    print(x.shape)
    u = np.array([[0.02],[0.02]])
    print(u.shape)

    # xdot = controller.f(x,u)
    # print(xdot)
    # print(xdot.shape)

    A,B = controller.linearize(x,u)
    print(A)
    print(A.shape)
    print(B) 
    print(B.shape)

    print("Perturbation!! ")
    A,B = controller.linearize_numerical_perturbation(x,u)
    print(A)
    print(A.shape)
    print(B) 
    print(B.shape)


    # linear quadratic regulator
    Q = np.eye((4))
    R = 1e-2*np.eye((2))

    K,S,E = control.lqr(A,B,Q,R)
    print(K)
    print(K.shape)

    C=np.eye((4))
    print(C)
    v = controller.tracking_input(A, B, K, C, ja)
    print(v)
    print(v.shape)
