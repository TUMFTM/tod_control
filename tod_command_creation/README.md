# tod_command_creation
This package provides nodes to calculate the primary and secondary control commands and send them to the vehicle.
It launches the `OperatorCmdCreator` node and transmits the commands to the vehicle.
Furthermore, force feedback commands are generated to position the operator steering wheel similar to vehicle steering wheel.


# Dependencies
  * ROS Packages: see `package.xml`


# Documentation

## OperatorCommandCreator
If connected to vehicle, calculates primary and secondary control commands from joystick message.

**Published Topics:**
 * `/Operator/CommandCreation/primary_control_command` ([tod_msgs/PrimaryControlCmd](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/PrimaryControlCmd.msg))
 * `/Operator/CommandCreation/secondary_control_cmd` ([tod_msgs/SecondaryControlCmd](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/SecondaryControlCmd.msg))

**Subscriptions:**
 * `/Operator/InputDevices/joystick` ([sensor_msgs/Joy](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Joy.html)) Contains the state of buttons and axes of the current input device.
 * `/Operator/Manager/status_msg` ([tod_msgs/Status](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/Status.msg)) Contains the connection Status to the Vehicle.


## ForceFeedbackController
When receiving vehicle data with vehicle steering wheel angle position, computes force feedback commands to position operator steering wheel similar to vehicle steering wheel.

**Published Topics:**
 * `/Operator/CommandCreation/force_feedback` ([std_msgs/Float64](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html)) Force feedback value.

**Subscriptions:**
 * `/Operator/VehicleBridge/vehicle_data` ([tod_msgs/VehicleData](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/VehicleData.msg)) Contains vehicle steering wheel angle position.
 * `/Operator/InputDevices/joystick` ([sensor_msgs/Joy](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Joy.html)) Contains operator steering wheel angle position..


## PrimaryCmdSender/Receiver
Transmission of `primary_control_cmd` ([tod_msgs/PrimaryControlCmd](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/PrimaryControlCmd.msg)) from operator to vehicle, if connected to vehicle.

## SecondaryCmdSender/Receiver
Transmission of `secondary_control_cmd` ([tod_msgs/SecondaryControlCmd](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/SecondaryControlCmd.msg)) from operator to vehicle, if connected to vehicle.
