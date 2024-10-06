### Takeaways

#### Force controller
* Desired quantities to control in the end affector space can be mapped to force vectors which are then mapped to joint space to achieve the desired control
* Sometimes its not easy to map quantites to control to the control space
* Not robust to external disturbance
* Needs hand tuning of controller behaviour like oscillations and stead state error

#### State space control
* Necesaary condition is the dynamic equation of the system, for an N-link manipulator lagrangian energy method is a good way to derive this
* A non linear system like the manipulator needs to be linearised about an operating point to apply the state space control theory
* Linearisation can be achieved using taylor series expansion and also numerical perturbation, in practice numerical perturbation is way easier to implement and the result is identical from both the methods
* End affector force cannot be modelled in the dynamic equation of the manipulator since the state space tracking controller compensates those internal forces to achieve zero velocities in the x direction
* End affector force needs a feedforward term in the control law along with the state space tracking feedback term
* Does not need hand tuning of the controller behaviour, more robust to external disturbance
