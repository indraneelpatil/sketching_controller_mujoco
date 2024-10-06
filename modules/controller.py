
# Created by Indraneel on 21/07/2024
import math
import numpy as np


def force_control(kinematics, ja, jw, problem_parameters):
    """
    Implements a basic force controller
    1. Calculates the required force vector on the end affector of the robot arm
    2. Maps the end affector force to joint space torques
    3. Sanity checks on the torques and applies the torques

    Parameters: 
    ______________
    kinematics
    ja :                    current joint angles  np.ndarray 2x1
    jt :                    current joint torques  np.ndarray 2x1
    problem_parameters:     problem parameters dict

    Returns:
    ______________
    control_signal:          new motor torques to apply np.ndarray 2x1

    Algorithm :
    _____________
    1. Imagine a point inside the wall and reduce position error to the point to ensure the arm always stays on the wall
    2. Use a force of K*(position error to this point) on the end affector of the robotic arm where K is calculated as (desired force on the wall)/(total distance to this point)
    3. Once the position error is zero or less than a threshold just use the desired force 
    4. Once contact is made with the wall, add another force component to give end affector velocity along the wall
    5. Add both these force components to get the total joint torques needed on each motor
    """
    # Initialise controller
    end_affector_position = kinematics.forward_kinematics(ja)
    end_affector_position_x = end_affector_position[0][0]
    end_affector_position_y = end_affector_position[1][0]
    print('ja {}'.format(ja))
    print('End affector position at {}'.format(end_affector_position))

    # Arbitrary chosen point inside the wall 
    distance_of_point_from_the_wall = 1.0

    # Calculate constant such that when the arm is at this distance
    # applied force should be desired force
    desired_force = problem_parameters['desired_force_on_the_wall_N']
    K_position_correction = desired_force/distance_of_point_from_the_wall

    # calculate current distance from this point
    tracking_point_x_coordinate = problem_parameters['distance_from_base_to_wall_m'] + distance_of_point_from_the_wall
    position_error = tracking_point_x_coordinate-end_affector_position_x

    # Force to correct position error to the point
    end_affector_force = K_position_correction*position_error
    desired_force_vec_along_x = np.array([[end_affector_force],[0.0]])

    jacobian = kinematics.jacobian(ja)
    jacobian_t = jacobian.T

    # If made contact with the wall apply force along the wall 
    # to produce velocity along the wall
    desired_force_vec_along_y = np.array([[0.0],[0.0]])
    distance_from_the_wall = problem_parameters['distance_from_base_to_wall_m']-end_affector_position_x
    if abs(distance_from_the_wall)<0.1:
        print('Made contact with the wall')
        k_weight_velocity = 5.0

        desired_velocity_vec =  np.array([[0.0],[-problem_parameters['desired_velocity_along_the_wall_ms']]])
        current_velocity_vec = jacobian @ jw
        desired_force_vec_along_y = k_weight_velocity*(desired_velocity_vec - current_velocity_vec)

    net_end_affector_force = desired_force_vec_along_x+desired_force_vec_along_y
    
    #  Map end affector force to torques
    control_signal = jacobian_t @ net_end_affector_force
    
    print(' ')
    print(' force vec along x error \n {}'.format(desired_force_vec_along_x))
    print(' force vec along y error \n {}'.format(desired_force_vec_along_y))
    print(' Net Control force (End affector) \n {}'.format(net_end_affector_force))
    print(' Control signal (Joint Space) \n {}'.format(control_signal))

    return control_signal
