# Shared Velocity Control Approach
Shared Velocity Control (SVC) approach, capable of overriding the commanded velocity from the operator. As described in the paper, referenced in the top-level README of the package, the approach consists of two stages. In the first stage, a trajectory tree is sampled. In the second second, a velocity profile is optimized. The first instance of this profile is commanded as the desired velocity to the vehicle. The steering angle from the operator is just passed through. The velocity profile is computed using MPC. The formulation is solved through [acados](https://github.com/acados/acados). The C code is generated from the Matlab interface, done in `acados_matlab/`.

**Publications:**
 * /Vehicle/SharedControl/avoided_obstacles [`tod_msgs/ObjectList`] The filtered list of avoided obstacles (the nearest five).  
 * /Vehicle/SharedControl/predicted_polygon [`tod_msgs/ColoredPolygon`] Polyon for the visual interface of the operator.
 * /Vehicle/SharedControl/primary_control_cmd [`tod_msgs/PrimaryControlCmd`] Control command from shared controller.
 * /Vehicle/SharedControl/svc_log [`tod_shared_control/SvcLog`] Log from shared controller.
 * /Vehicle/SharedControl/trajectory_tree [`visualization_msgs/MarkerArray`] Sampled trajectory tree for visualization.

**Subscriptions:**
 * /Vehicle/Lidar/dummy_object_list [`tod_msgs/ObjectList`] List of obstacles to avoid. 
 * /Vehicle/Manager/status_msg [`tod_msgs/Status`] Status from manager. The control commands are only computed, when the operator is connected and the teleoperation is active.
 * /Vehicle/SharedControl/Shadow/primary_control_cmd [`tod_msgs/PrimaryControlCmd`]
 * /Vehicle/VehicleBridge/odometry [`nav_msgs/Odometry`] Current odometry from vehicle.
 * /Vehicle/VehicleBridge/vehicle_data [`tod_msgs/VehicleData`] Current state from vehicle.
 * /tf_static [`tf2_msgs/TFMessage`] Transforms for transforming the obstacles into the local vehicle frame.
