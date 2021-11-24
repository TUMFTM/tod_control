// Copyright 2020 Feiler

#pragma once

#include "grid_map_core/grid_map_core.hpp"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/PoseStamped.h"
#include "glm/vec3.hpp"

namespace tod_perception_modification::Geometry {

class Conversion {
public:
    inline static grid_map::Polygon PointCloud_to_GridMapPolygon(
            const sensor_msgs::PointCloud& pointCloud) {
        grid_map::Polygon tmpPolygon;
        std::for_each(std::begin(pointCloud.points), std::end(pointCloud.points),
            [&tmpPolygon](const geometry_msgs::Point32& point){
                tmpPolygon.addVertex(grid_map::Position(
                    (double) point.x, (double) point.y)); });
        return tmpPolygon;
    }

    inline static geometry_msgs::PoseStamped GLMvec3_to_PoseStamped(
            const glm::vec3& glmVec3) {
        geometry_msgs::PoseStamped tmpPoseStamped;
        tmpPoseStamped.pose.position.x = glmVec3.x;
        tmpPoseStamped.pose.position.y = glmVec3.y;
        tmpPoseStamped.pose.position.z = glmVec3.z;
        tmpPoseStamped.pose.orientation.w = 1.0;
        return tmpPoseStamped;
    }

    inline static glm::vec3 PoseStamped_to_GLMvec3(
            const geometry_msgs::PoseStamped& poseStamped) {
        glm::vec3 tmpVec3;
        tmpVec3.x = poseStamped.pose.position.x;
        tmpVec3.y = poseStamped.pose.position.y;
        tmpVec3.z = poseStamped.pose.position.z;
        return tmpVec3;
    }
};

} // namespace tod_helper::Geometry
