# tod_command_creation
This package provides nodes to calculate the primary and secondary control commands and send them to the vehicle.
It launches the `OperatorCmdCreator` node and transmits the commands to the vehicle.


# Dependencies
* ROS Packages:
    * roscpp
    * tod_msgs
    * tod_helper_functions
    * tod_network
    * tod_vehicle_config (exec only)


# Documentation

## OperatorCommandCreator
If connected to vehicle, calculates primary and secondary control commands from joystick message.

**Published Topics:**
* `/Operator/CommandCreation/primary_control_command` ([tod_msgs/PrimaryControlCmd](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/PrimaryControlCmd.msg))
* `/Operator/CommandCreation/secondary_control_cmd` ([tod_msgs/SecondaryControlCmd](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/SecondaryControlCmd.msg))


**Subscriptions:**
* `/Operator/InputDevices/joystick` ([sensor_msgs/Joy](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Joy.html)) Contains the state of buttons and axes of the current input device.
* `/Operator/Manager/status_msg` ([tod_msgs/Status](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/Status.msg)) Contains the connection Status to the Vehicle.


## OperatorPrimaryCommandSender
If connected to the vehicle, sends primary control commands to the vehicle.
**Subscriptions:**
* `/Operator/Manager/status_msg` ([tod_msgs/Status](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/Status.msg)) Contains the connection Status to the Vehicle.
* `/Operator/CommandCreation/primary_control_command` ([tod_msgs/PrimaryControlCmd](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/PrimaryControlCmd.msg))


## OperatorSecondaryCommandSender
If connected to the vehicle, sends primary control commands to the vehicle.
**Subscriptions:**
* `/Operator/Manager/status_msg` ([tod_msgs/Status](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/Status.msg)) Contains the connection Status to the Vehicle.
* `/Operator/CommandCreation/secondary_control_cmd` ([tod_msgs/SecondaryControlCmd](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/SecondaryControlCmd.msg))



## VehiclePrimaryCommandReceiver
Receives primary control commands from the operator
**Published Topics:**
* `/Operator/CommandCreation/primary_control_command` ([tod_msgs/PrimaryControlCmd](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/PrimaryControlCmd.msg))

## VehicleSecondaryCommandReceiver
Receives secondary control commands from the operator
**Published Topics:**
* `/Operator/CommandCreation/secondary_control_cmd` ([tod_msgs/SecondaryControlCmd](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/SecondaryControlCmd.msg))


