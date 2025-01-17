# Reach Controllers

This package provides ros2_control hardware interfaces for the Reach Robotics
manipulators.

## Alpha 5 Hardware Interface

A ros2_control hardware interface for the Reach Alpha 5 manipulator. The
hardware interface connects to the manipulator using serial at a baudrate of
115200 (non-configurable).

### Plugin Library

reach_hardware/Alpha5Hardware

### State Interfaces

* **/position
  * axis_a/position
  * axis_b/position
  * axis_c/position
  * axis_d/position
  * axis_e/position
* **/velocity
  * axis_a/velocity
  * axis_b/velocity
  * axis_c/velocity
  * axis_d/velocity
  * axis_e/velocity
* **/effort
  * axis_a/effort
  * axis_b/effort
  * axis_c/effort
  * axis_d/effort
  * axis_e/effort

[!NOTE]
> Joint torques are calculated using the approximation $\tau \approx G_r K_t I$,
> where $G_r$ is the joint gear ratio, $K_t$ is the joint torque constant, and
> I is the measured joint current.

### Command Interfaces

* **/position
  * axis_a/position
  * axis_b/position
  * axis_c/position
  * axis_d/position
  * axis_e/position
* **/velocity
  * axis_a/velocity
  * axis_b/velocity
  * axis_c/velocity
  * axis_d/velocity
  * axis_e/velocity
* **/effort
  * axis_a/effort
  * axis_b/effort
  * axis_c/effort
  * axis_d/effort
  * axis_e/effort

[!NOTE]
> Joint effort commands are converted into joint current commands using the
> approximation $I \approx \frac{\tau}{G_r K_t}$, where $G_r$ is the joint gear
> ratio, $K_t$ is the joint torque constant, and I is the measured joint
> current.

### Control Modes

* position control
* velocity control
* current (effort) control

### Parameters

* serial_port: The manipulator serial port (e.g., /dev/ttyUSB0)
* device_id: A given joint's device ID. This should be provided per joint.

[!WARNING]
> Prior to launching the Alpha 5 Hardware Interface, ensure that the serial
> port has been given read/write permissions.
