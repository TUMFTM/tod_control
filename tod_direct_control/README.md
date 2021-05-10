# tod_direct_control
The direct control commands are created in the tod_command_creation package. This node republishes the primary_control_command if Direct Control mode is selected.

# Dependencies
* ROS Packages:
    * tod_msgs

# Documentation
## VehicleForwardPrimaryCtrlCmd
This node republishes the primary_control_command if Direct Control mode is selected.

**Published Topics:**
* `/Vehicle/DirectControl/primary_control_command` ([tod_msgs/PrimaryControlCmd](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/PrimaryControlCmd.msg))

**Subscriptions:**
* `/Vehicle/CommandCreation/primary_control_command` ([tod_msgs/PrimaryControlCmd](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/PrimaryControlCmd.msg))
