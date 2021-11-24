// Copyright 2020 Feiler

#pragma once

#include "ros/ros.h"
#include <string>
#include <vector>
#include <map>
#include <functional>
#include "tod_msgs/Status.h"
#include "tod_msgs/Color.h"
#include "tod_msgs/Mesh.h"
#include "tod_msgs/KeyPress.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud.h"
#include "GLFW/glfw3.h"
#include "grid_map_msgs/GridMap.h"
#include <grid_map_ros/grid_map_ros.hpp>
#include "ObjectList.h"
#include "Conversion.h"
#include "Msg.h"
#include "GridMap.h"
#include "tod_helper/status/ModeChecker.h"
#include <algorithm>
#include <memory>
#include <chrono>
#include "tf2/utils.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tod_helper/status/ChangeDetector.h"
#include "StateMachine.h"
#include "tod_perception_modification/PercModOperatorRequest.h"
#include "tod_perception_modification/PercModVehicleResponse.h"
#include "tod_perception_modification/PercModOperatorApproval.h"

static tod_msgs::Color BLUE;
static tod_msgs::Color BRIGHTBLUE;
static tod_msgs::Color RED;
static tod_msgs::Color YELLOW;
static tod_msgs::Color GREEN;
static tod_msgs::Color SKIP;

const double GRID_CELL_SIZE_IN_M{ 0.1 };
const double OCCUPIED{ 0.1 }; // larger than that is occupied

class OperatorPerceptionModification {
    public:
        explicit OperatorPerceptionModification(ros::NodeHandle& nodeHandle);
        void process();
        bool controlModeIsChosen();
        void publishData();

    private:
        std::vector<ros::Subscriber> listOfSubscribers;
        std::map<std::string, ros::Publisher> mapOfPublishers;
        tod_msgs::ObjectList inputObjectList;
        tod_msgs::ObjectList modifiedObjectList;
        grid_map::GridMap inputGridMap;
        grid_map::GridMap modifiedGridMap;
        tod_msgs::Mesh outputObjectListAsMesh;
        tod_msgs::Mesh inputGridMapAsMesh;
        tod_msgs::Mesh outputGridMapAsMesh;
        nav_msgs::Odometry currentOdometry;
        sensor_msgs::PointCloud listOfClicks;
        bool statusIsTeleoperation { false };
        bool statusIsDisconnected { true };
        std::unique_ptr<StateMachine> percModStatus;
        grid_map::GridMap currentMarkedArea;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;
        std::vector<int> gotAllCallBacks;
        bool gotNewMouseClicks{ false };
        std::unique_ptr<tod_helper::Status::ModeChecker> controlModeChecker;
        tod_msgs::Status inputStatusMsg;
        std::unique_ptr<tod_helper::Status::TodStatusChangeDetector> statusChangeDetector;

        tod_perception_modification::PercModOperatorRequest operatorRequest;
        tod_perception_modification::PercModVehicleResponse vehicleResponse;
        tod_perception_modification::PercModOperatorApproval operatorApproval;

        void callbackStatusMsg(const tod_msgs::Status& msg);
        void callbackObjectList(const tod_msgs::ObjectList& msg);
        void callbackGridMap(const grid_map_msgs::GridMap& msg);
        void callbackGridMapVisualization(const tod_msgs::Mesh& msg);
        void callbackMouseClicks(const geometry_msgs::Point& msg);
        void callbackKeyPress(const tod_msgs::KeyPress& msg);
        void callbackOdometry(const nav_msgs::Odometry& msg);
        void addObjectToMesh(tod_msgs::Mesh& activeMesh,
            const std::vector<glm::vec3>& vectorToCorners,
            const unsigned int& index, const tod_msgs::Color& rgbColor);
        void addVertices(tod_msgs::Mesh& activeMesh,
            const tod_msgs::ObjectData& currentObject);
        void addVertices(tod_msgs::Mesh& activeMesh,
            const std::vector<glm::vec3>& vectorToCorners);
        void addIndices(std::vector<unsigned int>& indices,
            const unsigned int index, const int numberOfVerticesPerObject);
        void addColors(const tod_msgs::Color& rgbColor,
            std::vector<tod_msgs::Color>& colors, const int& numberOfColorsAdded);
        void writeRemainingGridMapIntoGridMapMesh(
            const double& radius, grid_map::GridMap& gridMap,
            tod_msgs::Mesh& gridMapMesh, const nav_msgs::Odometry& odometry);
        void initRosSubscribersAndPublishers(ros::NodeHandle& nodeHandle);
        void initCurrentMarkedArea(const std::string& frame_id);
        void initColors();
        void deleteClicksAndPubIt();
        void publishListOfClicks();

        void updateOperatorRequest();
        void publishOperatorRequestToBeSentToVehicle();
        void doSomethingWithObjects();

        void callbackVehicleResponse(
            const tod_perception_modification::PercModVehicleResponse& msg);
        void updateOperatorApproval();
        void publishOperatorApprovalToBeSentToVehicle();
        void tellVehicleToStop();
        void processClickOnEnter();
        void modifyGridMapVisualization();
        void deleteThisCellFromModifiedGridMap(
            const grid_map::Position& tmpPosition);
        bool enterWasPressed();
        void handleVelocityIncrement(const bool positiveIncrement);
        void alignObjectsWithFloor(std::vector<std::vector<glm::vec3>>& objectListAsVec3Mesh,
            const geometry_msgs::TransformStamped& transformFromLidarFrontIntoFTM);
};
