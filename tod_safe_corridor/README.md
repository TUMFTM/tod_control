# tod_safe_corridor

## Dependencies

see `package.xml`

## Quick Start

### Compile and Start

Follow the documentation in [teleoperated_driving](https://github.com/TUMFTM/teleoperated_driving). In `OperatorManagerWidget` klick on `Connect`, select `Safe Corridor` and click on `Start`. Select the `VirtualInputDevice`, shift gears (Up: "T", Down: "G") and start driving.

**Advice:** In `tod_safe_corridor.launch` set `verbose` for `tod_safety_gate` to `true`. Therefore, the safety gate will output information on the current state and state changes.

### Provoke a Safety Issue

`OperatorWatchdog` and `VehicleWatchdog` (package: `tod_safety_monigoring`) are monitoring multiple signals (e.g. Video Streams). Start vehicle.launch in mode `playbackSim` and deactivate a VideoStream in `Video Scene Manager` while driving. This will cause an issue in OperatorWatchdog and close the SafetyGate. The vehicle will stop within the visualized corridor.

## SafeTrajCreator

Subscribes to the operators control commands and creates a desired trajectory according to [Hoffmann et al.](#reference). Additionally the safe corridor is created according to [Hoffmann et al.](#reference). The color of the safe corridor corresponds to the worst state of the operator's and vehicle's safety gate (package: *tod_safety_monitoring*). (<span style="color:green">Open</span>, <span style="color:yellow">Warning</span>, <span style="color:red">Closed</span>).

**Parameters:**

`segmentLength:` Desired euclidean distance between 2 consecutive Poses (Default: 0.3)

`corridorBufferFront:` Virtually increases the vehicle length for corridor calculation (Default: 0.1)

`corridorBufferSide:` Virtually increases the vehicle width for corridor calculation (Default: 0.1)

`corridorLongitudinalOffset:` Adds an additional buffer to the corridor in driving direction (Default: 3.0)

`desiredDeceleration:` Desired deceleration of the calculated velocity profile for each trajectory (Default: 4.0)

`decelerationOffset:`  Duration until desired trajectory starts deceleration (Default: 0.2)

`lookaheadRatio:` Ratio that increases the length of the trajectory depending on the current velocity (e.g. lookahead ratio when using pure pursuit control) (Default: 0.0)

`minLookaheadDistance:` Minimum length of calculated trajectory (e.g. lookahead distance when using pure pursuit control) (Default: 0.0)

`frame_odom:` Odometry frame (Default: ftm)

`frame_rear_axle_footprint:` Frame of vehicles rear axle (Default: rear_axle_footprint)

*Vehicle Parameters:* Need to be specified in `tod_vehicle_config/vehicle_config/\<vehicleID\>`. If the desired Vehicle-ID is loaded into the Parameter Server, the vehicle parameters are automatically loaded using `tod_core`

**Published Topics:**

    * `trajectory_control_command` [tod_msgs/Trajectory]
    * `corridor` [tod_msgs/ColoredPolygon]

**Subscriptions:**

    * `/status_msg` [tod_msgs/Status]
    * `/odometry` [nav_msgs/Odometry]
    * `/primary_control_cmd` [tod_msgs/PrimaryControlCmd]
    * `/gate_state_operator` [tod_safety_monitoring/GateState]
    * `/gate_state_vehicle` [tod_safety_monitoring/GateState]

## OperatorTrajectorySender/Receiver

Transmission of `tod_msgs/Trajectory` from operator to vehicle, if connected to vehicle.

## Reference

The concept of this packages was published in:

> S. Hoffmann, D. Majstorović and F. Diermeyer
> **"Safe Corridor: A Trajectory-Based Safety Concept for Teleoperated Road Vehicles"**, 
> 2022 International Conference on Connected Vehicle and Expo (ICCVE), 2022, pp. 1-6, doi: 10.1109/ICCVE52871.2022.9742770.

    @INPROCEEDINGS{
        author={Hoffmann, Simon and Majstorović, Domagoj and Diermeyer, Frank},  booktitle={2022 International Conference on Connected Vehicle and Expo (ICCVE)}, 
        title={Safe Corridor: A Trajectory-Based Safety Concept for Teleoperated Road Vehicles}, 
        year={2022},
        pages={1-6},
        doi={10.1109/ICCVE52871.2022.9742770}
    }
