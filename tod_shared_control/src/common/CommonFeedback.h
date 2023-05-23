// Copyright 2021 Andreas Schimpe
#include <ros/ros.h>
#include <tod_msgs/Status.h>
#include <tod_msgs/VehicleData.h>
#include <tod_msgs/PrimaryControlCmd.h>
#include <nav_msgs/Odometry.h>
#include <tod_core/VehicleParameters.h>
#include <string>
#include <map>

namespace tod_shared_control {

class CommonFeedback {
public:
    explicit CommonFeedback(ros::NodeHandle &nodeHandle);
    void reset() { _primaryCtrlCmd.reset(); _odometry.reset(); _vehicleData.reset(); }
    void update_previous_in_teleoperation() { _previousInTeleoperation = in_teleoperation(); }

    bool in_shared_control_and_teleoperation() const { return in_shared_control() && in_teleoperation(); }
    bool in_shared_control() const { return (_status.vehicle_control_mode == tod_msgs::Status::CONTROL_MODE_SHARED); }
    bool in_teleoperation() const { return (_status.tod_status == tod_msgs::Status::TOD_STATUS_TELEOPERATION); }
    bool started_teleoperation() const { return (!_previousInTeleoperation && in_teleoperation()); }
    bool terminated_teleoperation() const { return (_previousInTeleoperation && !in_teleoperation()); }

    tod_msgs::PrimaryControlCmdConstPtr primary_ctrl_cmd() const { return _primaryCtrlCmd; }
    nav_msgs::OdometryConstPtr odometry() const { return _odometry; }
    tod_msgs::VehicleDataConstPtr vehicle_data() const { return _vehicleData; }

    void disable_control_cmd_feedback() { _subCtrlCmd.shutdown(); _needsControlCmd = false; }

    bool feedback_complete() const {
        bool ret = primary_ctrl_cmd() || !_needsControlCmd;
        return (ret && odometry() && vehicle_data());
    }

private:
    ros::NodeHandle _nh;
    std::string _nn;
    ros::Subscriber _subStatus, _subOdom, _subVehData, _subCtrlCmd;
    bool _needsControlCmd{true};

    tod_msgs::Status _status;
    bool _previousInTeleoperation{false};
    tod_msgs::VehicleDataConstPtr _vehicleData{nullptr};
    tod_msgs::PrimaryControlCmdConstPtr _primaryCtrlCmd{nullptr};
    nav_msgs::OdometryConstPtr _odometry{nullptr};
};

}; // namespace tod_shared_control
