#pragma once
#include <iostream>
#include <vector>
#include <math.h>
#include <cmath>
#include <algorithm>

struct VehicleParams {
    float LengthFrontAxle{0.0f}, LengthRearAxle{0.0f};
    float LengthFrontBumper{0.0f}, LengthRearBumper{0.0f};
    float Width{0.0f}, MaxDelta{0.0f}, MaxDDelta{0.0f};
};

static inline float deg2rad(const float deg) { return (deg * 3.1415f / 180.0f); }

static inline void calc_corners(const float xpos, const float ypos, const float heading,
                                const float distance, const float width,
                                float &xl, float &yl, float &xr, float &yr) {
    xl = xpos + distance * std::cos(heading) - width * std::sin(heading) / 2.0f;
    yl = ypos + distance * std::sin(heading) + width * std::cos(heading) / 2.0f;
    xr = xpos + distance * std::cos(heading) + width * std::sin(heading) / 2.0f;
    yr = ypos + distance * std::sin(heading) - width * std::cos(heading) / 2.0f;
}

static inline void calc_center(const float xpos1, const float ypos1,
                               const float xpos2, const float ypos2,
                               float &xCenter, float &yCenter) {
    xCenter = xpos1 + 0.5f * (xpos2 - xpos1);
    yCenter = ypos1 + 0.5f * (ypos2 - ypos1);
}

static inline float compute_beta(const float lf, const float lr, const float delta) {
    return std::atan(lr * std::tan(delta) / (lf + lr));
}

static inline float compute_curvature(const VehicleParams &vehParams, const float delta) {
    const float beta = compute_beta(vehParams.LengthFrontAxle, vehParams.LengthRearAxle, delta);
    return std::sin(beta) / vehParams.LengthRearAxle; // center of mass
}

struct TreeParams {
    float TperStep{0.0f};
    int NofSegments{0};
    float TperSegment{0.0f};
    bool NoSteeringInFinalSegment{false};
    int NofDDeltas;
    float TrajectoryTimeHorizon{0.0f};
};

struct State {
    float X{0.0f}, Y{0.0f}, Theta{0.0f}, Delta{0.0f}, Velocity{0.0f}, Progress{0.0f};
};

struct Input {
    float DDelta{0.0f}, Acceleration{0.0f};
    Input(const float ddelta = 0.0f, const float acceleration = 0.0f)
        : DDelta{ddelta}, Acceleration{acceleration} {}
};

struct Obstacle {
    float CenterX{0.0f}, CenterY{0.0f}, Heading{0.0f}, DimX{0.0f}, DimY{0.0f};
};

struct Trajectory {
    std::vector<State> States;
    std::vector<Input> Inputs;
    bool CollisionFree{true};
    float SafeProgress{0.0f};
    Trajectory(const int nPts) :
        States{std::vector<State>(nPts)},
        Inputs{std::vector<Input>(nPts-1, Input(0.0f, 0.0f))} {}

    std::vector<double> get_curvature_profile(const VehicleParams &vehicleParams) const {
        std::vector<double> curvatureProfile;
        curvatureProfile.reserve(States.size());
        for (const State &state : States) {
            curvatureProfile.push_back(compute_curvature(vehicleParams, state.Delta));
        }
        return curvatureProfile;
    }

    std::vector<double> get_steering_angle_profile() const {
        std::vector<double> steeringAngleProfile;
        steeringAngleProfile.reserve(States.size());
        for (const State &state : States) {
            steeringAngleProfile.push_back(state.Delta);
        }
        return steeringAngleProfile;
    }
};

typedef std::vector<Trajectory> TrajectoryTree;
typedef std::vector<Obstacle> ObstacleList;

class TrajectoryTreeCreator {
public:
    TrajectoryTreeCreator(const TreeParams &treeParams, const VehicleParams &vehicleParams);
    void plan_trees_from(const State &state, const float deceleration);
    float get_safe_progress_for(const ObstacleList &obstacles);
    void get_critical_profiles(std::vector<double> &curvatureProfile,
                               std::vector<double> &steeringAngleProfile);
    // getters
    int get_tree_size() const { return _trees.back().size(); }
    int get_trajectory_size() const { return _trees.back().front().States.size(); }
    const TrajectoryTree& get_tree() const { return  _trees.back(); }

private:
    TreeParams _treeParams;
    VehicleParams _vehicleParams;
    std::vector<float> _segmentDDeltas;
    std::vector<TrajectoryTree> _trees;
    int _ptsPerSegment;

    void advance_trajectory(const int segIdx, Input &input,
                            Trajectory &traj, bool isFinalSegmentWithConstantSteering);

public:
    void print_for_matlab(const ObstacleList &obstacles) const;
    static void print_obstacles_for_matlab(const ObstacleList &obstacles);
};

static inline void advance_bicycle(const VehicleParams &vehParams, const float dt,
                                   const State &state0, Input &input0, State &state1) {
    const float beta = compute_beta(vehParams.LengthFrontAxle, vehParams.LengthRearAxle, state0.Delta);
    state1.X = state0.X + dt * state0.Velocity * std::cos(state0.Theta + beta);
    state1.Y = state0.Y + dt * state0.Velocity * std::sin(state0.Theta + beta);
    state1.Theta = state0.Theta + dt * state0.Velocity * std::sin(beta) / vehParams.LengthRearAxle;

    const float newVel = state0.Velocity + dt * input0.Acceleration;
    if (newVel < 0.0f) {
        state1.Velocity = 0.0f;
        input0.Acceleration = (state1.Velocity - state0.Velocity) / dt;
    } else {
        state1.Velocity = newVel;
    }

    const float newDelta = state0.Delta + dt * input0.DDelta;
    if (std::abs(newDelta) >= vehParams.MaxDelta) {
        const float sign = (newDelta >= 0.0f) ? +1.0f : -1.0f;
        state1.Delta = sign * vehParams.MaxDelta;
        input0.DDelta = (state1.Delta - state0.Delta) / dt;
    } else {
        state1.Delta = newDelta;
    }

    float dx = state1.X - state0.X;
    float dy = state1.Y - state0.Y;
    state1.Progress = state0.Progress + std::sqrt(dx*dx + dy*dy);
}

static inline bool is_collision_free(const State &state, const ObstacleList &obstacles,
                                     const float lfb, const float lrb, const float width) {
#define APPROXIMATE_EGO_VEHICLE_AS_ELLIPSE 1
#if APPROXIMATE_EGO_VEHICLE_AS_ELLIPSE
    const float minor = std::sqrt(2.0f) * std::max(lfb, lrb);
    const float major = std::sqrt(2.0f) * width / 2.0f;
    for (const Obstacle &obs : obstacles) {
        static std::vector<float> obsXs(10), obsYs(10);
        calc_corners(obs.CenterX, obs.CenterY, obs.Heading, obs.DimX / 2.0f, obs.DimY,
                     obsXs.at(0), obsYs.at(0), obsXs.at(1), obsYs.at(1));
        calc_corners(obs.CenterX, obs.CenterY, obs.Heading, -obs.DimX / 2.0f, obs.DimY,
                     obsXs.at(2), obsYs.at(2), obsXs.at(3), obsYs.at(3));
        calc_center(obsXs.at(0), obsYs.at(0), obsXs.at(1), obsYs.at(1),
                    obsXs.at(4), obsYs.at(4));
        calc_center(obsXs.at(0), obsYs.at(0), obsXs.at(2), obsYs.at(2),
                    obsXs.at(5), obsYs.at(5));
        calc_center(obsXs.at(0), obsYs.at(0), obsXs.at(3), obsYs.at(3),
                    obsXs.at(6), obsYs.at(6));
        calc_center(obsXs.at(1), obsYs.at(1), obsXs.at(2), obsYs.at(2),
                    obsXs.at(7), obsYs.at(7));
        calc_center(obsXs.at(1), obsYs.at(1), obsXs.at(3), obsYs.at(3),
                    obsXs.at(8), obsYs.at(8));
        calc_center(obsXs.at(2), obsYs.at(2), obsXs.at(3), obsYs.at(3),
                    obsXs.at(9), obsYs.at(9));
        for (int i = 0; i < obsXs.size(); ++i) {
            const float obsX = obsXs.at(i);
            const float obsY = obsYs.at(i);
            const float dx = obsX - state.X;
            const float dy = obsY - state.Y;
            const float fac1 = std::cos(state.Theta) * dy - std::sin(state.Theta) * dx;
            const float fac2 = std::cos(state.Theta) * dx + std::sin(state.Theta) * dy;
            const float sum1 = ((std::sin(state.Theta) * fac2) / std::pow(minor, 2) + (std::cos(state.Theta) * fac1) / std::pow(major, 2)) * dy;
            const float sum2 = ((std::sin(state.Theta) * fac1) / std::pow(major, 2) - (std::cos(state.Theta) * fac2) / std::pow(minor, 2)) * dx;
            const float ellipse = sum1 - sum2;
            if (ellipse < 1.0f) {
                return false;
            }
        }
    }
#else
    // approximate ego vehicle as four shifted disks
    static const int nofDisks = 4;
    static const float endShift = 0.15f;
    static const float interShift = (1.0f - 2.0f * endShift) / float(nofDisks - 1);
    const float egoRearCenterX = state.X - std::cos(state.Theta) * lrb;
    const float egoRearCenterY = state.Y - std::sin(state.Theta) * lrb;
    const float egoFrontCenterX = state.X + std::cos(state.Theta) * lfb;
    const float egoFrontCenterY = state.Y + std::sin(state.Theta) * lfb;
    static std::vector<float> diskCentersX(nofDisks), diskCentersY(nofDisks);
    for (int i=0; i < nofDisks; ++i) {
        diskCentersX.at(i) = egoRearCenterX + (endShift + float(i) * interShift) * (egoFrontCenterX - egoRearCenterX);
        diskCentersY.at(i) = egoRearCenterY + (endShift + float(i) * interShift) * (egoFrontCenterY - egoRearCenterY);
    }
    const float r = 1.0f * width / 2.0f;

    // check if disks intersect with ellipse approximation of obstacle
    for (const auto& obstacle : obstacles) {
        const float obstacleEllipseMajor = std::sqrt(2.0f) * obstacle.DimX / 2.0f;
        const float obstacleEllipseMinor = std::sqrt(2.0f) * obstacle.DimY / 2.0f;
        for (int i=0; i < nofDisks; ++i) {
            const float dx = diskCentersX.at(i) - obstacle.CenterX;
            const float dy = diskCentersY.at(i) - obstacle.CenterY;
            const float ellipse =
                std::pow((dx*std::cos(obstacle.Heading)+dy*std::sin(obstacle.Heading))
                             / (obstacleEllipseMajor+r), 2.0) +
                std::pow((dx*std::sin(obstacle.Heading)-dy*std::cos(obstacle.Heading))
                             / (obstacleEllipseMinor+r), 2.0);
            if (ellipse <= 1.0f) {
                return false;
            }
        }
    }
#endif
    return true;
}
