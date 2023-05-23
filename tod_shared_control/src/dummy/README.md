# dummy nodes
The package contains the following dummy helper nodes for running the simulations. 

**Nodes:**
- [ObjectListPublisher](#objectlistpublisher)
- [TrajectoryGenerator](#trajectorygenerator)


# ObjectListPublisher
Publishes a list of dummy object for the shared control approaches to avoid. Through the compiler macro `RANDOM_OBSTACLE_FIELD` in the code, the list of objects is either a number of objects placed left and right alternatingly, or a random field of obstacles. 

**Publications:**
 * /Vehicle/Lidar/dummy_object_list (`tod_msgs/ObjectList`) The object list. 


# TrajectoryGenerator
Generates and publishes a straight path with constant velocity profile for the path tracking controllers to follow. Through this the (shadow) control actions from an operator are simulated. 

**Publications:**
 * /Vehicle/SharedControl/Shadow/trajectory (`tod_msgs/Trajectory`) The trajectory.
 