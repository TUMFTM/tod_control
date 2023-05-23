# network nodes
The package contains the following network nodes for transmitting data from the vehicle to the operator, and vice-versa. 

**Nodes:**
- [VehicleSenders](#vehiclesenders)
- [OperatorReceivers](#operatorreceivers)


# VehicleSenders
Node to send data from vehicle to operator. Subscribes to status message from manager to know when operator is connected. Only then, data is transmitted. 

**Subscriptions:** 
 * /Vehicle/Manager/status_msg [`tod_msgs/Status`] Status message from manager.  
 * /Vehicle/SharedControl/Shadow/primary_control_cmd [`tod_msgs/PrimaryControlCmd`] Control command from simulated operator. 
 * /Vehicle/SharedControl/avoided_obstacles [`tod_msgs/ObjectList`] List of avoided obstacles from shared control approach. 
 * /Vehicle/SharedControl/mpc_log [`tod_shared_control/MpcLog`] Log from MPC-based shared control approach (SSVC). 
 * /Vehicle/SharedControl/predicted_polygon [`tod_msgs/ColoredPolygon`] Polygon for operator's visual interface.  
 * /Vehicle/SharedControl/primary_control_cmd [`tod_msgs/PrimaryControlCmd`] Control command from shared control approach.  
 * /Vehicle/SharedControl/svc_log [`tod_shared_control/SvcLog`] Log from shared velocity control (SVC) approach. 


# OperatorReceivers
Node to receive data from vehicle. 

**Publications:** 
 * /Operator/SharedControl/Shadow/primary_control_cmd [`tod_msgs/PrimaryControlCmd`] Control command from simulated operaotr. 
 * /Operator/SharedControl/avoided_obstacles [`tod_msgs/ObjectList`] List of avoided obstacles from shared control approach.
 * /Operator/SharedControl/mpc_log [`tod_shared_control/MpcLog`] Log from MPC-based shared control approach (SSVC). 
 * /Operator/SharedControl/predicted_polygon [`tod_msgs/ColoredPolygon`] Polygon for operator's visual interface.
 * /Operator/SharedControl/primary_control_cmd [`tod_msgs/PrimaryControlCmd`] Control command from shared control approach.
 * /Operator/SharedControl/svc_log [`tod_shared_control/SvcLog`] Log from shared velocity control (SVC) approach. 
 * /Operator/SharedControl/*_paket_info [`tod_msgs/PaketInfo`] Paket info for each message on all received topics. 
