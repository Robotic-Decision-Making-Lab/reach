# This message defines the state for the multi-DoF impedance controller.

std_msgs/Header header

string[] dof_names

float64[] torques

float64[] position_errors

float64[] velocity_errors

float64[] p_terms

float64[] d_terms

float64[] outputs
