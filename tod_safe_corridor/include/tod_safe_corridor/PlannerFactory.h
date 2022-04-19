// Copyright 2021 Simon Hoffmann
#include "ConstantDeceleration.h"
#include "LongitudinalPlanner.h"
#include "LateralPlanner.h"
#include "ClothoidPlanner.h"
#include <memory>

enum class DecelerationProfile{
    TIME_LINEAR,
    WAY_LINEAR
};

enum class LateralBehavior{
    CLOTHOID,
    CIRCULAR_ARC
};

class LongPlannerFactory {
    public:
        LongPlannerFactory() = delete;
        static std::unique_ptr<LongitudinalPlanner> create(const DecelerationProfile profile) {
            switch (profile) {
            case DecelerationProfile::TIME_LINEAR :
                return std::make_unique<ConstantDeceleration>();
                break;
            case DecelerationProfile::WAY_LINEAR :
                ROS_ERROR_STREAM("Way Linear DecelerationProfile not implemented");
                return nullptr;
            default:
                ROS_ERROR_STREAM("Desired DecelerationProfile not implemented");
                return nullptr;
            }
        }
};

class LatPlannerFactory {
    public:
        LatPlannerFactory() = delete;
        static std::unique_ptr<LateralPlanner> create(const LateralBehavior behavior,
                std::shared_ptr<tod_core::VehicleParameters> vehParams, float segmentLength) {
            switch (behavior) {
            case LateralBehavior::CLOTHOID :
                return std::make_unique<ClothoidPlanner>(vehParams, segmentLength);
                break;
            case LateralBehavior::CIRCULAR_ARC :
                ROS_ERROR_STREAM("Circular Arc not implemented");
                return nullptr;
            default:
                ROS_ERROR_STREAM("Desired DecelerationProfile not implemented");
                return nullptr;
            }
        }
};
