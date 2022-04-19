// Copyright 2020 Simon Hoffmann
#include "LateralPlanner.h"

LateralPlanner::LateralPlanner(std::shared_ptr<tod_core::VehicleParameters> vehParams, float segmentLength)
        : _segmentLength(segmentLength), _vehParams(vehParams) { }
