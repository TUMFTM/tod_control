# Model Predictive Control-based Shared Controller
MPC-based Shared Steering and Velocity (SSVC) controller, capable of overriding the commanded steering and velocity from the operator. As described in the paper, referenced in the top-level README of the package, the approach is an MPC formulation with the objective to follow the control commands from the operator. When a collision is predicted, the approach deviates from these commands. The MPC formulation is solved through [acados](https://github.com/acados/acados). The C code is generated from the Matlab interface, done in `acados_matlab/`.

**Publications:**
 * /Vehicle/SharedControl/avoided_obstacles [`tod_msgs/ObjectList`] The filtered list of avoided obstacles (the nearest five).  
 * /Vehicle/SharedControl/predicted_polygon [`tod_msgs/ColoredPolygon`] Polyon for the visual interface of the operator.
 * /Vehicle/SharedControl/primary_control_cmd [`tod_msgs/PrimaryControlCmd`] Control command from shared controller.
 * /Vehicle/SharedControl/mpc_log [`tod_shared_control/MpcLog`] Log from shared controller.

**Subscriptions:**
 * /Vehicle/Lidar/dummy_object_list [`tod_msgs/ObjectList`] List of obstacles to avoid. 
 * /Vehicle/Manager/status_msg [`tod_msgs/Status`] Status from manager. The control commands are only computed, when the operator is connected and the teleoperation is active.
 * /Vehicle/SharedControl/Shadow/primary_control_cmd [`tod_msgs/PrimaryControlCmd`]
 * /Vehicle/VehicleBridge/odometry [`nav_msgs/Odometry`] Current odometry from vehicle.
 * /Vehicle/VehicleBridge/vehicle_data [`tod_msgs/VehicleData`] Current state from vehicle.
 * /tf_static [`tf2_msgs/TFMessage`] Transforms for transforming the obstacles into the local vehicle frame.
