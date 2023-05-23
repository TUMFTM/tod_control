#include "TrajectoryTree.h"

TrajectoryTreeCreator::TrajectoryTreeCreator(const TreeParams &treeParams, const VehicleParams &vehicleParams)
    : _treeParams{treeParams}, _vehicleParams{vehicleParams},
    _ptsPerSegment{int(std::round(_treeParams.TperSegment / _treeParams.TperStep))} {
    float ddeltastep = 2 * _vehicleParams.MaxDDelta / float(_treeParams.NofDDeltas - 1);
    for (int i=0; i < _treeParams.NofDDeltas; ++i) {
        _segmentDDeltas.emplace_back(-_vehicleParams.MaxDDelta + float(i) * ddeltastep);
    }
    int nTrees = _treeParams.NofSegments;
    if (_treeParams.NoSteeringInFinalSegment) {
        nTrees -= 1;
    }

    for (int sIdx = 0; sIdx < nTrees; ++sIdx) {
        int nTjsInSegment = std::pow(_segmentDDeltas.size(), sIdx + 1);
        int ptsPerTrajectory = 1 + (sIdx + 1) * _ptsPerSegment;
        if (_treeParams.NoSteeringInFinalSegment && sIdx == nTrees-1) {
            ptsPerTrajectory += _ptsPerSegment;
        }
        _trees.push_back(TrajectoryTree(nTjsInSegment, Trajectory(ptsPerTrajectory)));
    }
}

void TrajectoryTreeCreator::plan_trees_from(const State &state, const float deceleration) {
    for (int iIdx = 0; iIdx < _segmentDDeltas.size(); ++iIdx) {
        Input input(_segmentDDeltas.at(iIdx), deceleration);
        Trajectory& tj2adv = _trees.front().at(iIdx);
        State& first = tj2adv.States.front();
        first = state;
        advance_trajectory(0, input, tj2adv, false);
    }

    for (int tIdx = 1; tIdx < _trees.size(); ++tIdx) {
        const TrajectoryTree& tree2advFrom = _trees.at(tIdx-1);
        TrajectoryTree& tree2adv = _trees.at(tIdx);
        int nofAdvancedTrajectories = 0;
        for (const Trajectory& tj2advFrom : tree2advFrom) {
            for (const float ddelta : _segmentDDeltas) {
                Input input(ddelta, deceleration);
                Trajectory& tj2adv = tree2adv.at(nofAdvancedTrajectories);
                std::copy(tj2advFrom.Inputs.begin(), tj2advFrom.Inputs.end(), tj2adv.Inputs.begin());
                std::copy(tj2advFrom.States.begin(), tj2advFrom.States.end(), tj2adv.States.begin());
                bool isFinalSegmentWithConstantSteering = (tIdx == _trees.size()-1)
                                                          && _treeParams.NoSteeringInFinalSegment;
                advance_trajectory(tIdx, input, tj2adv, isFinalSegmentWithConstantSteering);
                ++nofAdvancedTrajectories;
            }
        }
    }
}

float TrajectoryTreeCreator::get_safe_progress_for(const ObstacleList &obstacles) {
    float _safeProgressGlobal = 1000.0f;
    std::for_each(_trees.back().begin(), _trees.back().end(), [&](Trajectory &tj2check) {
        for (const State &state : tj2check.States) {
            if ( !(tj2check.CollisionFree = is_collision_free(
                      state, obstacles, _vehicleParams.LengthFrontBumper,
                      _vehicleParams.LengthRearBumper, _vehicleParams.Width)) ) {
                _safeProgressGlobal = std::min(_safeProgressGlobal, tj2check.SafeProgress);
                break;
            }
            tj2check.SafeProgress = state.Progress;
        }
    });
    // assign minimum collision free distance, even if all tjs in tree are collision free
    if (_safeProgressGlobal > 100.0f) {
        _safeProgressGlobal = _trees.back().front().SafeProgress;
    }
    return _safeProgressGlobal;
}

void TrajectoryTreeCreator::get_critical_profiles(std::vector<double> &curvatureProfile,
                                                  std::vector<double> &steeringAngleProfile) {
    auto trajectory = (_trees.back().front().States.front().Delta <= 0.0)
                          ? _trees.back().front() : _trees.back().back();
    curvatureProfile = trajectory.get_curvature_profile(_vehicleParams);
    steeringAngleProfile = trajectory.get_steering_angle_profile();
}

void TrajectoryTreeCreator::advance_trajectory(const int segIdx, Input &input,
                                               Trajectory &traj, bool isFinalSegmentWithConstantSteering) {
    int first = segIdx * _ptsPerSegment + 1;
    int last = (segIdx+1) * _ptsPerSegment;
    for (int t = first; t <= last; ++t) {
        advance_bicycle(_vehicleParams, _treeParams.TperStep,
                        traj.States.at(t-1), input, traj.States.at(t));
        traj.Inputs.at(t-1) = input;
    }
    if (isFinalSegmentWithConstantSteering) {
        const Input input2(0.0f, input.Acceleration);
        for (int t = last+1; t <= last + _ptsPerSegment; ++t) {
            advance_bicycle(_vehicleParams, _treeParams.TperStep,
                            traj.States.at(t-1), input, traj.States.at(t));
            traj.Inputs.at(t-1) = input2;
        }
    }
}

void TrajectoryTreeCreator::print_for_matlab(const ObstacleList &obstacles) const {
    const TrajectoryTree& tree2print = _trees.back();
    for (int i=0; i < tree2print.size(); ++i) {
        printf("state_trajs{%d}=[", i+1);
        for (const auto &z : tree2print.at(i).States) {
            printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f ; ",
                   z.X, z.Y, z.Theta, z.Delta, z.Velocity, z.Progress);
        }
        printf("];");
    }

    for (int i=0; i < tree2print.size(); ++i) {
        printf("input_trajs{%d}=[", i+1);
        for (const auto &u : tree2print.at(i).Inputs) {
            printf("%.3f, %.3f;", u.DDelta, u.Acceleration);
        }
        printf("];");
    }

    for (int i=0; i < tree2print.size(); ++i) {
        printf("collision_free{%d}=%d;", i+1, tree2print.at(i).CollisionFree);
    }

    for (uint i = 0; i < obstacles.size(); ++i) {
        const Obstacle &obs = obstacles.at(i);
        printf("obstacles{%d}=[%.3f, %.3f, %.3f, %.3f, %.3f]; ",
               i+1, obs.CenterX, obs.CenterY, obs.Heading, obs.DimX, obs.DimY);
    }

    printf("distances_collision_free=[");
    for (int i=0; i < tree2print.size(); ++i) {
        printf("%.3f, ", tree2print.at(i).SafeProgress);
    }
    printf("];");

    printf("ego=[%.3f, %.3f, %.3f];", _vehicleParams.LengthFrontAxle, _vehicleParams.LengthRearAxle, _vehicleParams.Width);
}
