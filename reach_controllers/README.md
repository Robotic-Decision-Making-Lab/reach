# Reach Controllers

This package provides a collection of controllers for the Reach Robotics
manipulators.

## Impedance Controller

A chainable dynamic controller designed for use as an inner controller in an
MPC framework. The control law implemented is as follows:

$\tau_j^{\text{ref}} = \tau_j^{\text{ff}} + \textbf{K}_p(q_j^{\text{ref}} - q_j) + \textbf{K}_d(\dot{q}_j^{\text{ref}} - \dot{q}_j)$
