# Model Predictive Control-based Trajectory Following Controller
MPC-based path tracking controller (ptc) following a path with a velocity profile. The control command is only computed, when the shared control mode is selected and the teleoperation is active. The MPC formulation is solved through [acados](https://github.com/acados/acados). The C code is generated from the Matlab interface, done in `acados_matlab/`.

**Publications:**
 * /Vehicle/SharedControl/Shadow/predictions [`visualization_msgs/MarkerArray`] Predictions of the controller for visualization. 
 * /Vehicle/SharedControl/Shadow/primary_control_cmd [`tod_msgs/PrimaryControlCmd`] Control command from controller. 
 * /Vehicle/SharedControl/Shadow/reference [`visualization_msgs/MarkerArray`] Reference points of the controller for visualization.
 * /Vehicle/SharedControl/Shadow/solver_time [`std_msgs/Float32`] Solver time of controller.
 * /Vehicle/SharedControl/Shadow/tracking_error [`std_msgs/Float32`] Lateral tracking error of controller.

**Subscriptions:**
 * /Vehicle/Manager/status_msg [`tod_msgs/Status`] Status from manager. 
 * /Vehicle/SharedControl/Shadow/trajectory [`tod_msgs/Trajectory`] Path with velocity to follow.
 * /Vehicle/VehicleBridge/odometry [`nav_msgs/Odometry`] Current odometry from vehicle.
 * /Vehicle/VehicleBridge/vehicle_data [`tod_msgs/VehicleData`] Current state from vehicle.
