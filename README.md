### Takeaways

#### Mujoco
* World model of the 2R manipulator arm in front of a sketching board/ wall
* Need to enable contact forces between manipulator and the board
* Need to add sensors to end affector to measure our controller quantities

#### Force controller
* Desired quantities to control in the end affector space can be mapped to force vectors which are then mapped to joint space to achieve the desired control
* Sometimes its not easy to map quantites to control to the control space
* Not robust to external disturbance
* Needs hand tuning of controller behaviour like oscillations and stead state error

#### State space control
* Necesaary condition is the dynamic equation of the system, for an N-link manipulator lagrangian energy method is a good way to derive this, math can get complicated
* A non linear system like the manipulator needs to be linearised about an operating point to apply the state space control theory
* Linearisation can be achieved using taylor series expansion and also numerical perturbation, in practice numerical perturbation is way easier to implement and the result is identical from both the methods
* End affector force cannot be modelled in the dynamic equation of the manipulator since the state space tracking controller compensates those internal forces to achieve zero velocities in the x direction
* End affector force needs a feedforward term in the control law along with the state space tracking feedback term
* Does not need hand tuning of the controller behaviour, more robust to external disturbance


#### Reference videos
* LQR tracking https://www.youtube.com/watch?v=X3Dfy8H4Inc
* Mujoco world model https://www.youtube.com/watch?v=j1nCeqtfySQ
* LQR regulator around a static equilibrium point (including numerical perturbation method of linearistion) https://www.youtube.com/watch?v=KvNmOMk8pYs&list=PLc7bpbeTIk75dgBVd07z6_uKN1KQkwFRK&index=27
* Taylor series linearisation https://www.youtube.com/watch?v=owv_2YdO8rA

#### Special thanks
* Prof. Pranav Bhounsule's Mujoco lecture series on youtube
* My girlfriend for all the no travel weekends
