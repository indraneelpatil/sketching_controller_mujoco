
# Created by Indraneel on 21/07/2024
import math
import numpy as np

class Kinematics:
    def __init__(self,robot_parameters):
        """
        Provides kinematics for a 2 link robotic arm given robot parameters

        Parameters:
        _____________
        robot_parameters: link lengths l1 and l2

        """
        self.robot_parameters = robot_parameters
    

    def forward_kinematics(self, joint_angles):
        """
        Converts the joint space to task space

        Parameters:
        _____________
        joint_angles:  np.ndarray 2x1

        Returns:
        __________
        end_affector_coordinates: np.ndarray 2x1
        """

        l1 = self.robot_parameters['l1']
        l2 = self.robot_parameters['l2']
        theta1 = joint_angles[0][0]
        theta2 = joint_angles[1][0]

        end_affector_coordinates = np.array([[l1*math.cos(theta1) + l2*math.cos(theta1+theta2)], 
                                            [l1*math.sin(theta1) +l2*math.sin(theta1+theta2)]])

        return end_affector_coordinates
    
    def inverse_kinematics(self,end_affector_coordinates):
        """
        Converts the task space to joint space

        Parameters:
        _____________
        end_affector_coordinates:  np.ndarray 2x1

        Returns:
        __________
        joint_angles: np.ndarray 2x1
        """
        l1 = self.robot_parameters['l1']
        l2 = self.robot_parameters['l2']
        x = end_affector_coordinates[0][0]
        y = end_affector_coordinates[1][0]

        # Check if the point is within reach
        if (x**2 + y**2) > (l1 + l2)**2:
            temp = "Target point {} {} is out of reach.".format(x,y)
            raise ValueError(temp)

        # Make sure in cosine domain
        temp1 = (x*x+y*y-l1*l1-l2*l2)/(2*l1*l2)
        temp1 = min(1.0,temp1)
        temp1 = max(-1.0,temp1)

        # Elbow-down configuration (use positive for elbow-up)
        theta2 = -math.acos(temp1)
        theta1 = math.atan2(y,x)-math.atan2(l2*math.sin(theta2),l1+l2*math.cos(theta2))
        joint_angles = np.array([[theta1],
                                 [theta2]])
        
        return joint_angles

    def jacobian(self, joint_angles):
        """
        Calculates the jacobian of the arm at a given joint angle configuration

        Parameters:
        _____________
        joint_angles:  np.ndarray 2x1

        Returns:
        __________
        jacobian: np.ndarray 2x2

        """
        l1 = self.robot_parameters['l1']
        l2 = self.robot_parameters['l2']
        theta1 = joint_angles[0][0]
        theta2 = joint_angles[1][0]

        jacobian = np.array([ [-l1*math.sin(theta1)-l2*math.sin(theta1+theta2),-l2*math.sin(theta1+theta2)], 
                            [l1*math.cos(theta1)+l2*(math.cos(theta1+theta2)), l2*math.cos(theta1+theta2)]])
        return jacobian


if __name__ == "__main__":
    robot_parameters = {
        'l1': 0.3,
        'l2': 0.3
    }
    kinematics = Kinematics(robot_parameters)

    ja =  np.array([[0.1],[-0.2]])
    print(ja)
    forward_kinematics = kinematics.forward_kinematics(ja)
    print(forward_kinematics)
    print(forward_kinematics.shape)

    inverse_kinematics = kinematics.inverse_kinematics(forward_kinematics)
    print(inverse_kinematics)
    print(inverse_kinematics.shape)

    jacobian = kinematics.jacobian(ja)
    print(jacobian)
    print(jacobian.shape)
