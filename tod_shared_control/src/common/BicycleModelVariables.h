// Copyright 2021 Andreas Schimpe
#pragma once
#include <math.h>

namespace tod_shared_control {

struct BicycleInput {
    double SteeringRate;
    double Acceleration;

    BicycleInput(double steeringRate = 0.0, double acceleration = 0.0) :
        SteeringRate{steeringRate},
        Acceleration{acceleration} { }
};


struct BicycleCommand {
    double Steering;
    double Velocity;
    BicycleInput input;

    BicycleCommand(double steering = 0.0, double velocity = 0.0) :
        Steering{steering},
        Velocity{velocity} { }
};

struct BicycleState {
    double PosX;
    double PosY;
    double Heading;
    BicycleCommand Cmd;

    BicycleState(double posX = 0.0, double posY = 0.0, double heading = 0.0,
             double steering = 0.0, double velocity = 0.0) :
        PosX{posX},
        PosY{posY},
        Heading{heading},
        Cmd{steering, velocity} { }

    double distance_to(const BicycleState &pt) const {
        double dx = pt.PosX - PosX;
        double dy = pt.PosY - PosY;
        return std::sqrt(dx*dx + dy*dy);
    }
};

}; // namespace tod_shared_control
