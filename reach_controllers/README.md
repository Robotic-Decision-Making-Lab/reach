# Reach Controllers

This package provides a collection of controllers for Reach Robotics
manipulators.

## Impedance Controller

A chainable dynamic controller. Given a joint $j$ with position $q_j$ and
velocity $\dot{q}_j$, the implemented control law is given as follows

```math
\tau_j^{\text{ref}} = \tau_j^{\text{ff}} \mu_j^{\text{ff}} + \textbf{K}_p(q_j^{\text{ref}} - q_j) + \textbf{K}_d(\dot{q}_j^{\text{ref}} - \dot{q}_j),
```

where $\tau_j^{\text{ff}}$ is a feedforward joint torque, \mu_j^{\text{ff}} is
a feedforward joint friction term, $\textbf{K}_p$ is the desired joint
stiffness, and $\textbf{K}_d$ is the desired joint damping.

This control law is commonly used as an inner controller in an MPC framework [^1] [^2].

[^1]: I. Dadiotis, A. Laurenzi, and N. Tsagarakis. "Whole-body MPC for highly redundant legged manipulators: experimental evaluation with a 37 DoF dual-arm quadruped," in *IEEE International Conference on Humanoid Robots (Humanoids)*, 2023.
[^2]: J. -P. Sleiman, F. Farshidian, M. V. Minniti and M. Hutter, "A Unified MPC Framework for Whole-Body Dynamic Locomotion and Manipulation," in *IEEE Robotics and Automation Letters*, vol. 6, no. 3, pp. 4688-4695, July 2021.

### Plugin Library

reach_controllers/ImpedanceController

### References

* Target joint position $q_j^\text{ref}$ [rad]
* Target joint velocity $\dot{q}_j^\text{ref}$ [rad/s]
* Target joint torque $\tau_j^{\text{ff}}$ [Nm]

### State Feedback

* Measured joint position $q_j$ [rad]
* Measured joint velocity $\dot{q}_j$ [rad/s]

### Commands

The output of this controller is a desired joint torque $\tau_j^{\text{ref}}$ [Nm]

### Subscribers

* impedance_controller/multi_dof_impedance_command [reach_msgs::msg::MultiDofImpedanceCommand]

### Publishers

* impedance_controller/status [reach_msgs::msg::MultiDofImpedanceStateStamped]

### Parameters

* enable_parameter_update_without_reactivation: If enabled select parameters will be dynamically updated while the controller is running. [bool]
* joints: List of joints controlled by the impedance controller. [string array]
* reference_names: The names of the reference controllers. This can be used to configure command interfaces in chained mode. [string array]
* gains (provided per joint):
  * friction: The joint friction. [double]
  * stiffness: The desired joint stiffness. [double]
  * damping: The desired joint damping. [double]
