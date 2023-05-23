std_msgs/Header header

# controller input
float64 deltaRef
float64 velocityRef
float64 deltaMin
float64 deltaMax
float64 ddeltaMin
float64 ddeltaMax
float64 accelerationMin
float64 accelerationMax
tod_msgs/ObjectData[] objectList
nav_msgs/Odometry odometry

# controller output
float64 deltaCtrler
float64 velocityCtrler
float64 ddeltaCtrler
float64 accelerationCtrler
tod_msgs/ColoredPolygon predictedPolygon

# debug
int32 acados_status
int32 acados_sqp_iterations
int32 acados_solver_time_ms
int32 nx
int32 nu
int32 N
float64[] xTraj
float64[] uTraj
