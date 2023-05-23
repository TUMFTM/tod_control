std_msgs/Header header

# controller input
float64 desiredAcceleration
float64 desiredVelocity
float64 desiredRWA
float64 accelerationMin
float64 accelerationMax
float64 velocityMax
float64 lateralAccelerationMax
float64 jerkMax
tod_msgs/ObjectData[] objectList
nav_msgs/Odometry odometry

# planner and collision checker output
float64 safeProgress
float64[] criticalCurvatureProfile
float64[] criticalSteeringAngleProfile
# trajectory tree trajectories (optional, en-/disable in launch file)
# visualization_msgs/MarkerArray trajsMarker
tod_shared_control/FloatArray[] trajsXPoses
tod_shared_control/FloatArray[] trajsYPoses
tod_shared_control/FloatArray[] trajsHeadings
tod_shared_control/FloatArray[] trajsSteeringAngles
tod_shared_control/FloatArray[] trajsVelocities
tod_shared_control/FloatArray[] trajsProgressValues
tod_shared_control/FloatArray[] trajsSteeringRates
tod_shared_control/FloatArray[] trajsLonAccs
bool[] trajsCollisionFree
float32[] trajsSafeProgress

# velocity optimizer output
float64 jerkCmd
float64 accelerationCmd
float64 velocityCmd
float64 rwaCmd
float64[] timeProfile
float64[] jerkProfile
float64[] accelerationProfile
float64[] velocityProfile
float64[] progressProfile
float64[] lateralAccelerationProfile

# debug
float32 trajectoryPlannerTimeMs
float32 collisionCheckerTimeMs
float32 velocityOptimizerTimeMs
float32 totalControllerTimeMs

# vehicle params
float32 vehicleLengthFront
float32 vehicleLengthRear
float32 vehicleLengthFrontBumper
float32 vehicleLengthRearBumper
float32 vehicleWidth
