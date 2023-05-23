std_msgs/Header header

# current
float64 currentVelocity
float64 currentRWA
nav_msgs/Odometry odometry

# desired
float64 desiredVelocity
tod_msgs/TrajectoryPoint trackingPoint

# controller
tod_msgs/PrimaryControlCmd ctrlCmd
float64 lookahead
float64 lateralError
float64 headingError
float64 trackingError
