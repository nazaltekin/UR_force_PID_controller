# UR_force_PID_controller
Design of force PID controller on UR5 

HYBRID COMPLIANCE CONTROL OF COLLABORATIVE ROBOT

SUMMARY

Robots are valuable part of the manufacturing in industry for their capability to be precise, consistent, and fast. Therefore, robotics is an essential piece of Industry 4.0. In processes that need high speed/high force robots are preferred, because they are programmed to bring maximum efficiency. Various control schemes are developed for automation of any kind of robot and the control is done via sensors and actuators. Compliant control scheme, allow to adjust force and torque exerted by environment onto robot manipulator during operations.

In this thesis an indirect hybrid position/force controller is developed using a UR10 collaborative robot, for deburring processes. Every deburring process may require specific force control parameters due to variations in workpiece material, form, burr heights etc. The developed control scheme aims to provide flexibility by making these control parameters adjustable. So that, same manipulator can be used with different components that require specific deburring parameters. Whereas the built-in force controller in the robot is closed to the end-user.

First, kinematic modelling of general robot structures is explained. UR10 kinematics is shared using Denavit - Hartenberg parameters. Next, dynamic modeling methods are explained which defines the relationship between robot motion, and manipulator forces/torques. Jacobian concept is summarized, because CB3 series Universal Robots do not have a pre-installed F/T transducer at end-effector to measure the forces and torques. The controller is estimating these values on tool, through Jacobian and motor torque constants at each joint. During testing, noise is observed in the data collected, hence low-pass filters are introduced. It is concluded that Bessel filter is most suitable due to its characteristics in magnitude and phase responses through experiments. Motion control is explored through various control schemes in free and complaint environments. Compliant schemes are used in applications where force and position are controlled at different directions in a coordinate frame. Hybrid position/force control schemes are explained, and block diagrams are shared.

An indirect hybrid compliant control scheme is created to be used with UR controller. The control scheme includes proportional, integral and derivative gains that are determined through testing. The control scheme is tested on a flat surface and a curved pipe. Reaction force and displacement results are compared with results from tests with the built-in force controller.

In conclusion, comparison results are discussed. Shortcomings of the study and their impact are shared. Future possible improvements are recommended.
