# Created by Indraneel on 12/08/2024
import numpy as np
from modules.kinematics import Kinematics
from modules.controller import force_control
import matplotlib.pyplot as plt

import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os

xml_path = 'mujoco/models/2_link_planar_arm.xml' #xml file (assumes this is in the same folder as this file)
simend = 10 #simulation time

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

# Todo next
# 1. Find some way to track velocity and force on end affector
# 2. Forces look incorrect, maybe try removing the friction?
# 3. Revise lqr
# 4. Implement same controller in lqr and compare


def init_controller(model,data):
    #initialize the controller here. This function is called once, in the beginning
    global kinematics,problem_parameters

    # define constants
    robot_parameters = {
        'l1': 1.0,
        'l2': 1.0,
    }

    problem_parameters = {
        'distance_from_base_to_wall_m' : 1.5, #in m
        'desired_force_on_the_wall_N' : 0.5, #in N
        'desired_velocity_along_the_wall_ms' : 0.25, #in m/s
    }

    #intialise modules
    kinematics = Kinematics(robot_parameters)

def controller(model, data):
    #put the controller here. This function is called inside the simulation.
    global kinematics,problem_parameters

    ja= np.array([[data.qpos[0]],[data.qpos[1]]])
    jw = np.array([[data.qvel[0]],[data.qvel[1]]])

    # calculate control input
    jt = force_control(kinematics,ja,jw,problem_parameters)
    data.ctrl[0] = jt[0][0]
    data.ctrl[1] = jt[1][0]

    # disturbance torque
    tau_disturb_mean = 0
    tau_disturb_dev = 5
    tau_d0 = np.random.normal(tau_disturb_mean, tau_disturb_dev)
    tau_d1 = np.random.normal(0*tau_disturb_mean, 0*tau_disturb_dev)
    data.qfrc_applied[0] = tau_d0
    data.qfrc_applied[1] = tau_d1

def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)
    

#get the full path
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath

# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)                # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options

# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()
window = glfw.create_window(1200, 900, "Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# initialize visualization data structures
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# install GLFW mouse and keyboard callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)


# Example on how to set camera configuration
cam.azimuth = 89.88401446742182 ; cam.elevation = -88.95090525153114 ; cam.distance =  6.063267137946909
cam.lookat =np.array([ 0.0 , 0.0 , 0.0 ])

#initialize the controller
init_controller(model,data)

#set the controller
mj.set_mjcb_control(controller)

def main():

    # define initial state
    data.qpos[0] = 1.4
    data.qpos[1] = -0.78

    # Record the data
    num_timesteps = 556
    end_affector_poses = np.zeros((3,num_timesteps))
    end_affector_velocities = np.zeros((3,num_timesteps))
    end_affector_force_vec = np.zeros((3,num_timesteps))
    joint_torques = np.zeros((2,num_timesteps))
    time = np.zeros(num_timesteps)
    i=0

    # run the controller
    while not glfw.window_should_close(window):
        time_prev = data.time

        while (data.time - time_prev < 1.0/60.0):
            mj.mj_step(model, data)

        print('x pos {}'.format(data.sensordata[0:3]))
        print('x vel {}'.format(data.sensordata[3:6]))
        print('x force {}'.format(data.sensordata[6:]))

        tip_body_name = "dummy body"
        tip_id = model.body(tip_body_name).id
        xmat_global = data.xmat[tip_id].reshape(3, 3)
        force_vec_local =  data.sensordata[6:]
        force_vec_global = np.dot(xmat_global, force_vec_local)

        # Save the data
        end_affector_poses[:,i] = data.sensordata[0:3]
        end_affector_velocities[:,i] = data.sensordata[3:6]
        end_affector_force_vec[:,i] = force_vec_global
        joint_torques[:,i] = data.ctrl[0:2]

        time[i] = data.time

        i=i+1

        if (data.time>=simend):
            break;

        # get framebuffer viewport
        viewport_width, viewport_height = glfw.get_framebuffer_size(
            window)
        viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

        # Update scene and render
        mj.mjv_updateScene(model, data, opt, None, cam,
                        mj.mjtCatBit.mjCAT_ALL.value, scene)
        mj.mjr_render(viewport, scene, context)

        # swap OpenGL buffers (blocking call due to v-sync)
        glfw.swap_buffers(window)

        # process pending GUI events, call GLFW callbacks
        glfw.poll_events()

    # Plot joint torques
    plt.figure()
    plt.plot(time, joint_torques[0, :], label='Joint 1 Torque')
    plt.plot(time, joint_torques[1, :], label='Joint 2 Torque')
    plt.xlabel('Time (s)')
    plt.ylabel('Torque (Nm)')
    plt.title('Joint Torques over Time')
    plt.legend()
    plt.grid()

    # Plot end affector poses
    plt.figure()
    plt.plot(time, end_affector_poses[0, :], label='End effector pose X')
    plt.plot(time, end_affector_poses[1, :], label='End affector pose Y')
    plt.xlabel('Time (s)')
    plt.ylabel('End effector pose (m)')
    plt.title('End affector pose over time')
    plt.legend()
    plt.grid()

    # Plot end affector velocity
    plt.figure()
    plt.plot(time, end_affector_velocities[0, :], label='End effector velocity X')
    plt.plot(time, end_affector_velocities[1, :], label='End affector velocity Y')
    plt.xlabel('Time (s)')
    plt.ylabel('End effector Velocities (m/s)')
    plt.title('End affector velocities over time')
    plt.legend()
    plt.grid()

    # Plot end affector forces
    plt.figure()
    plt.plot(time, end_affector_force_vec[0, :], label='End effector force X')
    plt.plot(time, end_affector_force_vec[1, :], label='End affector force Y')
    plt.xlabel('Time (s)')
    plt.ylabel('End effector force (N)')
    plt.title('End affector force over time')
    plt.legend()
    plt.grid()

    plt.show()

def controller_test():
    robot_parameters = {
        'l1': 0.3,
        'l2': 0.3
    }
    kinematics = Kinematics(robot_parameters)

    jt =  np.array([[0.0],[0.0]])
    ja =  np.array([[0.78],[-0.78]])
    jw =  np.array([[0.0],[0.0]])

    problem_parameters = {
        'distance_from_base_to_wall_m' : 0.6, #in m
        'desired_force_on_the_wall_N' : 0.5, #in N
        'desired_velocity_along_the_wall_ms' : 0.1, #in m/s
    }

    control_signal = force_control(kinematics,ja,jw,problem_parameters)

if __name__ == "__main__":
    main()
    glfw.terminate()




