# Feedback Linearization-based Path Tracking Controller
Controller following a path using a feedback linearization (FBL)-based control law. The control command is only computed, when the shared control mode is selected and the teleoperation is active.

**Publications:** 
 * /Vehicle/SharedControl/Shadow/fbl_log [`tod_shared_control/FblLog`] Log from controller.
 * /Vehicle/SharedControl/Shadow/primary_control_cmd [`tod_msgs/PrimaryControlCmd`] Control command from controller. 

**Subscriptions:** 
 * /Vehicle/Manager/status_msg [`tod_msgs/Status`] Status message from manager. 
 * /Vehicle/SharedControl/Shadow/trajectory [`tod_msgs/Trajectory`] Path with velocity profile to follow.
 * /Vehicle/VehicleBridge/odometry [`nav_msgs/Odometry`] Current odometry from vehicle. 
 * /Vehicle/VehicleBridge/vehicle_data [`tod_msgs/VehicleData`] Current state from vehicle.
