// Copyright 2020 Feiler

#pragma once
#include <iostream>
#include <vector>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtc/matrix_transform.hpp>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>

#include "tod_msgs/ObjectData.h"
#include "tod_msgs/ObjectList.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "grid_map_core/grid_map_core.hpp"
#include "Conversion.h"
#include "nav_msgs/Odometry.h"
#include "ros/time.h"

namespace TodHelper {

class ObjectList {
    public:
        ObjectList() = default;

        //        6  ---  7
        //      /       /
        //    2  ---  3
        //
        //        5  ---  4
        //     /       /
        //    1  ---  0       z x
        //                    |/
        //               y ---
        static std::vector<glm::vec3> getVectorsToCorners(
            const tod_msgs::ObjectData& object) {
            // declaration for better readability
            glm::vec3 rearRightLow;
            glm::vec3 rearLeftLow;
            glm::vec3 rearLeftHigh;
            glm::vec3 rearRightHigh;
            glm::vec3 frontRightLow;
            glm::vec3 frontLeftLow;
            glm::vec3 frontLeftHigh;
            glm::vec3 frontRightHigh;

            // shorter names
            float width{ object.dimY };
            float height{ object.dimZ };
            float length{ object.dimX };

            // if ( object.objectRefPoint == static_cast<int>(
            //                                         ReferencePoint::OBJECT_MIDDLE) ) {
                rearRightLow = glm::vec3(-length/2, -width/2, 0.0f);
                rearLeftLow = glm::vec3(-length/2, width/2, 0.0f);
                rearLeftHigh = glm::vec3(-length/2, width/2, height);
                rearRightHigh = glm::vec3(-length/2, -width/2, height);
                frontRightLow = glm::vec3(length/2, -width/2, 0.0f);
                frontLeftLow = glm::vec3(length/2, width/2, 0.0f);
                frontLeftHigh = glm::vec3(length/2, width/2, height);
                frontRightHigh = glm::vec3(length/2, -width/2, height);
            // } else if (object.objectRefPoint == static_cast<int>(
            //                                         ReferencePoint::REAR_RIGHT) ) {
            //     rearRightLow = glm::vec3(0.0f, 0.0f, 0.0f);
            //     rearLeftLow = glm::vec3(0.0f, width, 0.0f);
            //     rearLeftHigh = glm::vec3(0.0, width, height);
            //     rearRightHigh = glm::vec3(0.0f, 0.0f, height);
            //     frontRightLow = glm::vec3(length, 0.0f, 0.0f);
            //     frontLeftLow = glm::vec3(length, width, 0.0f);
            //     frontLeftHigh = glm::vec3(length, width, height);
            //     frontRightHigh = glm::vec3(length, 0.0f, height);
            // } else if (object.objectRefPoint == static_cast<int>(
            //                                         ReferencePoint::REAR_LEFT) ) {
            //     rearRightLow = glm::vec3(0.0f, -width, 0.0f);
            //     rearLeftLow = glm::vec3(0.0f, 0.0f, 0.0f);
            //     rearLeftHigh = glm::vec3(0.0, 0.0f, height);
            //     rearRightHigh = glm::vec3(0.0f, -width, height);
            //     frontRightLow = glm::vec3(length, -width, 0.0f);
            //     frontLeftLow = glm::vec3(length, 0.0f, 0.0f);
            //     frontLeftHigh = glm::vec3(length, 0.0f, height);
            //     frontRightHigh = glm::vec3(length, -width, height);
            // }

            // push back into vector
            std::vector<glm::vec3> vectorToCorners;
            vectorToCorners.push_back(rearRightLow);
            vectorToCorners.push_back(rearLeftLow);
            vectorToCorners.push_back(rearLeftHigh);
            vectorToCorners.push_back(rearRightHigh);
            vectorToCorners.push_back(frontRightLow);
            vectorToCorners.push_back(frontLeftLow);
            vectorToCorners.push_back(frontLeftHigh);
            vectorToCorners.push_back(frontRightHigh);

            // get translation and rotation matrix
            glm::mat4 translation = glm::translate(glm::mat4(1.0f),
                        glm::vec3(object.distCenterX, object.distCenterY, 0.0f));
            glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), object.yawAngle,
                        glm::vec3(0.0f, 0.0f, 1.0f));

            // transform
            std::for_each(vectorToCorners.begin(), vectorToCorners.end(),
                [&rotation, &translation](glm::vec3& vector){
                    glm::vec4 tmpVector =
                        translation * rotation * glm::vec4(vector, 1.0f);
                    vector = glm::vec3(tmpVector.x, tmpVector.y, tmpVector.z);
                });

            return vectorToCorners;
        }

        static std::vector<glm::vec3> transformObject(
                const std::vector<glm::vec3>& objectAsVectors,
                const geometry_msgs::TransformStamped& transformFromTo) {
            std::vector<glm::vec3> transformedObjects;
            for ( auto& vertex : objectAsVectors ) {
                geometry_msgs::PoseStamped outPoint;
                geometry_msgs::PoseStamped inPoint = tod_perception_modification::Geometry::Conversion::
                    GLMvec3_to_PoseStamped(vertex);
                tf2::doTransform(inPoint, outPoint, transformFromTo);
                glm::vec3 tmpPoint = tod_perception_modification::Geometry::Conversion::PoseStamped_to_GLMvec3(outPoint);
                transformedObjects.push_back(tmpPoint);
            }
            return transformedObjects;
        }

        static std::vector<std::vector<glm::vec3>> createVectorOfObjectMeshes(
                const tod_msgs::ObjectList& objectListMsg) {
            std::vector<std::vector<glm::vec3>> objectListMesh;
            std::for_each(std::begin(objectListMsg.objectList),
                        std::end(objectListMsg.objectList),
                [&objectListMesh] (const tod_msgs::ObjectData& object) {
                    objectListMesh.push_back(TodHelper::ObjectList::getVectorsToCorners(object));
                });
            return objectListMesh;
        }

        static std::vector<std::vector<glm::vec3>> transformVectorOfObjectMeshes(
                const std::vector<std::vector<glm::vec3>>& objectListMeshIn,
                const geometry_msgs::TransformStamped& transformFromTo) {
            std::vector<std::vector<glm::vec3>> objectListMeshOut;
            for ( auto& object : objectListMeshIn ) {
                objectListMeshOut.push_back(transformObject(object, transformFromTo));
            }
            return objectListMeshOut;
        }

        static grid_map::Polygon createFloorPolygonFromVectors(
                const std::vector<glm::vec3>& objectAsVector) {
            grid_map::Polygon objectAsPolygon;
            objectAsPolygon.addVertex(grid_map::Position(
                objectAsVector.at(0).x, objectAsVector.at(0).y));
            objectAsPolygon.addVertex(grid_map::Position(
                objectAsVector.at(1).x, objectAsVector.at(1).y));
            objectAsPolygon.addVertex(grid_map::Position(
                objectAsVector.at(5).x, objectAsVector.at(5).y));
            objectAsPolygon.addVertex(grid_map::Position(
                objectAsVector.at(4).x, objectAsVector.at(4).y));
            return objectAsPolygon;
        }


        static void resetAndMoveCurrentMarkedArea(
                grid_map::GridMap& gridMap, const nav_msgs::Odometry& odometry) {
            gridMap.setTimestamp(ros::Time::now().toNSec());
            gridMap[gridMap.getLayers().front()].setConstant(0.0f);
            gridMap.move(grid_map::Position(
                odometry.pose.pose.position.x, odometry.pose.pose.position.y));
        }

        static void fillGridWithClicks(
                grid_map::GridMap& currentMarkedArea,
                const sensor_msgs::PointCloud& listOfClicks) {
            grid_map::Polygon cornersOfMarkedArea =
                tod_perception_modification::Geometry::Conversion::PointCloud_to_GridMapPolygon(listOfClicks);
            grid_map::PolygonIterator iterator(currentMarkedArea, cornersOfMarkedArea);
            grid_map::Position tmpPosition;
            for ( iterator; !iterator.isPastEnd(); ++iterator ) {
                currentMarkedArea.getPosition(*iterator, tmpPosition);
                if ( currentMarkedArea.isInside(tmpPosition) ) {
                    currentMarkedArea.at(currentMarkedArea.getLayers().at(0), *iterator) = 1.0;
                }
            }
        }

        static bool objectOverlapsWithArea(
                    const grid_map::GridMap& gridMap,
                    const grid_map::Polygon& polygon) {
            bool overlap = false;
            grid_map::PolygonIterator polygonIterator(gridMap, polygon);
            int numberOfIterations { 0 };
            for ( polygonIterator; !polygonIterator.isPastEnd(); ++polygonIterator ) {
                grid_map::Position tmpPosition;
                gridMap.getPosition(*polygonIterator, tmpPosition);
                if ( gridMap.at(gridMap.getLayers().at(0), *polygonIterator) == 1.0 ) {
                    overlap = true;
                }
                ++numberOfIterations;
            }
            return overlap;
        }

        static void removeRespectiveObjects(
        tod_msgs::ObjectList& modifiedObjectList,
        const std::vector<int>& indicesToBeDeleted) {
            for ( auto reverseIterator = indicesToBeDeleted.rbegin();
            reverseIterator != indicesToBeDeleted.rend(); ++reverseIterator ) {
                int indexToBeDeleted = *reverseIterator;
                modifiedObjectList.objectList.erase(
                    modifiedObjectList.objectList.begin() + indexToBeDeleted);
            }
        }
};

} // namespace TodHelper
