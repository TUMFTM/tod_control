# tod_pure_pursuit

This package provides a path tracking controller based on the pure pursuit algorithm.

## Dependencies

see `package.xml`

## Quick Start

```bash
catkin build tod_pure_pursuit
source devel/setup.bash
roslaunch tod_pure_pursuit tod_pure_pursuit.launch
```

## PathTrackingControlNode

Based on the vehicle position and a desired trajectory, a primary_control_commands is created.
**Published Topics:**

* `/primary_control_command` [tod_msgs/PrimaryControlCmd]

**Subscriptions:**

* `/Vehicle/Manager/status_msg` [tod_msgs/Status]
* `/Vehicle/VehicleBridge/odometry` [nav_msgs/Odometry]
* `/Vehicle/<control_mode>/trajectory_control_command` [tod_msgs/Trajectory]

## Reference

The pure pursuit algorithm is based on the following document:

> R Craig Coulter,
> **"Implementation of the pure pursuit path tracking algorithm"**, 
> DTIC Document, 1992

    @techreport{coulter1992implementation,
    title={Implementation of the pure pursuit path tracking algorithm},
    author={Coulter, R Craig},
    year={1992},
    institution={Carnegie-Mellon UNIV Pittsburgh PA Robotics INST}
    }
