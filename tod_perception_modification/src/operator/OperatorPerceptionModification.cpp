// Copyright 2020 Feiler

#include "OperatorPerceptionModification.h"
#include "tod_helper/status/ControlModeChecker.h"
#include "PercModStatus.h"
#include <utility>
#include <cstdlib>
#include "Transform.h"
#include "std_msgs/Bool.h"
#include <chrono>

OperatorPerceptionModification::OperatorPerceptionModification(
                                            ros::NodeHandle& nodeHandle) :
        tfListener(tfBuffer) {
    listOfClicks.header.frame_id = "ftm"; //TODO(johannes): given by visual+odom --> info should come from there
    initRosSubscribersAndPublishers(nodeHandle);
    initCurrentMarkedArea("ftm");
    initColors();
    controlModeChecker = std::make_unique<tod_helper::Status::ControlModeChecker>(
        tod_msgs::Status::CONTROL_MODE_PERCEPTION_MODIFICATION, &inputStatusMsg);
    statusChangeDetector = std::make_unique<tod_helper::Status::TodStatusChangeDetector>(
        &inputStatusMsg, tod_msgs::Status::TOD_STATUS_TELEOPERATION);
    percModStatus = std::make_unique<PercModStatus>();
}

void OperatorPerceptionModification::initRosSubscribersAndPublishers(ros::NodeHandle& nodeHandle) {
    mapOfPublishers.insert(std::pair<std::string, ros::Publisher>(
        "pubObjectListAsMesh",
        nodeHandle.advertise<tod_msgs::Mesh>("/output_object_list_as_mesh", 5)));
    mapOfPublishers.insert(std::pair<std::string, ros::Publisher>(
        "pubGridMapAsMesh",
        nodeHandle.advertise<tod_msgs::Mesh>("/grid_map_modified", 5)));
    mapOfPublishers.insert(std::pair<std::string, ros::Publisher>(
        "pubMouseClicksForVisualization",
        nodeHandle.advertise<sensor_msgs::PointCloud>("/Operator/PerceptionModification/MouseClickList", 5)));
    mapOfPublishers.insert(std::pair<std::string, ros::Publisher>(
        "pubMouseClicksForAreaVisualization",
        nodeHandle.advertise<sensor_msgs::PointCloud>(
            "/Operator/PerceptionModification/MouseClickListForAreaVisualization", 5)));
    mapOfPublishers.insert(std::pair<std::string, ros::Publisher>(
        "operator_request",
        nodeHandle.advertise<tod_perception_modification::PercModOperatorRequest>
        ("operator_request", 5)));
    mapOfPublishers.insert(std::pair<std::string, ros::Publisher>(
        "operator_approval",
        nodeHandle.advertise<tod_perception_modification::PercModOperatorApproval>
        ("operator_approval", 5)));
    mapOfPublishers.insert(std::pair<std::string, ros::Publisher>(
        "operator_increment_velocity",
        nodeHandle.advertise<std_msgs::Bool>
        ("operator_increment_velocity", 5)));

    listOfSubscribers.push_back(nodeHandle.subscribe("/status_msg", 5,
                    &OperatorPerceptionModification::callbackStatusMsg, this));
    listOfSubscribers.push_back(nodeHandle.subscribe("/object_list", 5,
                    &OperatorPerceptionModification::callbackObjectList, this));
    listOfSubscribers.push_back(nodeHandle.subscribe("/grid_map", 5,
                    &OperatorPerceptionModification::callbackGridMap, this));
    listOfSubscribers.push_back(nodeHandle.subscribe("/grid_map_visualization", 5,
                    &OperatorPerceptionModification::callbackGridMapVisualization, this));
    listOfSubscribers.push_back(nodeHandle.subscribe("/input_mouse_clicks", 5,
                    &OperatorPerceptionModification::callbackMouseClicks, this));
    listOfSubscribers.push_back(nodeHandle.subscribe("/input_key_press", 5,
                    &OperatorPerceptionModification::callbackKeyPress, this));
    listOfSubscribers.push_back(nodeHandle.subscribe("/odometry", 5,
                    &OperatorPerceptionModification::callbackOdometry, this));
    listOfSubscribers.push_back(nodeHandle.subscribe("vehicle_response", 5,
                    &OperatorPerceptionModification::callbackVehicleResponse, this));

    for ( int iterator = 0; iterator != listOfSubscribers.size(); ++iterator ) {
        gotAllCallBacks.push_back(0);
    }
    printf("size of listOfCallbacks %i\n", (int)listOfSubscribers.size());
}

void OperatorPerceptionModification::initCurrentMarkedArea(const std::string& frame_id) {
    currentMarkedArea.add("occupancy");
    currentMarkedArea.setFrameId(frame_id);
    currentMarkedArea.setGeometry(grid_map::Length(20, 20), GRID_CELL_SIZE_IN_M);
}

void OperatorPerceptionModification::initColors() {
    BLUE.r = 0.0f;
    BLUE.g = 10.0/255.0f;
    BLUE.b = 1.0f;

    BRIGHTBLUE.r = 0.0f;
    BRIGHTBLUE.g = 100.0/255.0f;
    BRIGHTBLUE.b = 1.0f;

    RED.r = 1.0f;
    RED.g = 0.0f;
    RED.b = 0.0f;

    YELLOW.r = 1.0f;
    YELLOW.g = 165.0f/255.0f;
    YELLOW.b = 0.0f;

    GREEN.r = 1.0f;
    GREEN.g = 1.0f;
    GREEN.b = 0.0f;
}

void OperatorPerceptionModification::process() {
    if ( statusChangeDetector->statusChanged() ) {
        deleteClicksAndPubIt();
    }
    if ( !statusIsTeleoperation ) {
        percModStatus->resetState();
    }
    if ( statusIsDisconnected ) {
        return;
    }
    if ( gotNewMouseClicks ) {
        gotNewMouseClicks = false;
        publishListOfClicks();
    }
    static bool firstTimeWithoutObjects { true };
    if ( !inputObjectList.objectList.empty() ) {
        doSomethingWithObjects();
        firstTimeWithoutObjects = true;
    } else {
        if ( firstTimeWithoutObjects ) {
            ROS_ERROR("object list is empty");
            firstTimeWithoutObjects = false;
        }
    }
    static bool firstTimeWithoutGridMap { true };
    if ( !inputGridMap.getLayers().empty() ) {
        modifyGridMapVisualization();
        firstTimeWithoutGridMap = true;
    } else {
        if ( firstTimeWithoutGridMap ) {
            ROS_ERROR("grid map does not have layers yet");
            firstTimeWithoutGridMap = false;
        }
    }
}

void OperatorPerceptionModification::doSomethingWithObjects() {
    modifiedObjectList = inputObjectList;
    TodHelper::Msg::clearMeshMsg(outputObjectListAsMesh);
    TodHelper::ObjectList::resetAndMoveCurrentMarkedArea(currentMarkedArea,
                                    currentOdometry);
    geometry_msgs::TransformStamped transformFromLidarFrontIntoFTM;
    if ( !TodHelper::getTransformToTargetFromSource(listOfClicks.header.frame_id,
    inputObjectList.header.frame_id, transformFromLidarFrontIntoFTM, tfBuffer) ) {
        return;
    }
    std::vector<std::vector<glm::vec3>> objectListAsVec3LidarFrame =
        TodHelper::ObjectList::createVectorOfObjectMeshes(inputObjectList);
    std::vector<std::vector<glm::vec3>> objectListAsVec3Mesh =
        TodHelper::ObjectList::transformVectorOfObjectMeshes(
        objectListAsVec3LidarFrame, transformFromLidarFrontIntoFTM);
    alignObjectsWithFloor(objectListAsVec3Mesh, transformFromLidarFrontIntoFTM);
    std::vector<tod_msgs::Color>
        howToTreatRespectiveObject(inputObjectList.objectList.size(), RED);
    if ( listOfClicks.points.size() > 2 ) {
        TodHelper::ObjectList::fillGridWithClicks(currentMarkedArea, listOfClicks);
        for ( int index = 0; index != inputObjectList.objectList.size(); ++index ) {
            grid_map::Polygon objectAsPolygon = TodHelper::ObjectList::
                createFloorPolygonFromVectors(objectListAsVec3Mesh.at(index));
            if ( TodHelper::ObjectList::objectOverlapsWithArea(currentMarkedArea,
            objectAsPolygon) ) {
                if ( enterWasPressed() ) {
                    howToTreatRespectiveObject.at(index) = SKIP;
                } else {
                    howToTreatRespectiveObject.at(index) = BRIGHTBLUE;
                }
            }
        }
    }
    int minus = 0;
    for ( int index = 0; index != modifiedObjectList.objectList.size(); ++index ) {
        if ( howToTreatRespectiveObject.at(index) == SKIP ) {
            ++minus;
        } else {
            addObjectToMesh(outputObjectListAsMesh, objectListAsVec3Mesh.at(index),
                index-minus, howToTreatRespectiveObject.at(index));
        }
    }
    mapOfPublishers["pubObjectListAsMesh"].publish(outputObjectListAsMesh);

    bool printIfAllCallbacksAreArrived{ false };
    if ( printIfAllCallbacksAreArrived ) {
        for ( auto elem : gotAllCallBacks ) {
            printf("elem %i\n", elem);
        }
        printf("\n");
    }
}

void OperatorPerceptionModification::alignObjectsWithFloor(
        std::vector<std::vector<glm::vec3>>& objectListAsVec3Mesh,
        const geometry_msgs::TransformStamped& transformFromLidarFrontIntoFTM) {
    for ( auto& object : objectListAsVec3Mesh ) {
        double height = std::abs(object.at(0).z - object.at(2).z);
        object.at(0).z = 0.0;
        object.at(1).z = 0.0;
        object.at(4).z = 0.0;
        object.at(5).z = 0.0;
        object.at(2).z = height;
        object.at(3).z = height;
        object.at(6).z = height;
        object.at(7).z = height;
    }
}

void OperatorPerceptionModification::modifyGridMapVisualization() {
    TodHelper::Msg::clearMeshMsg(outputGridMapAsMesh);
    // "modified" --> differently colored if deleted and therefore
    //not colored again
    modifiedGridMap = inputGridMap;

    // check if perception modification should do something
    if ( listOfClicks.points.size() > 2 ) {
        grid_map::Position tmpPosition;
        double halfCellSize = inputGridMap.getResolution()/2;
        grid_map::Polygon cornersOfMarkedArea =
            tod_perception_modification::Geometry::Conversion::PointCloud_to_GridMapPolygon(listOfClicks);
        grid_map::PolygonIterator markedAreaIterator(inputGridMap,
                                        cornersOfMarkedArea);
        for ( markedAreaIterator; !markedAreaIterator.isPastEnd(); ++markedAreaIterator ) {
            inputGridMap.getPosition(*markedAreaIterator, tmpPosition);
            if ( !inputGridMap.isInside(tmpPosition) ) {
                continue;
            }
            if ( inputGridMap.at(inputGridMap.getLayers().at(0), *markedAreaIterator)
                                                        == 1.0 ) {
                deleteThisCellFromModifiedGridMap(tmpPosition);
                if ( enterWasPressed() ) {
                    continue; // "deleted", if not in planning anymore
                }
                TodHelper::GridMap::addRectangleIndicesTo(outputGridMapAsMesh);
                inputGridMap.getPosition(*markedAreaIterator, tmpPosition);
                TodHelper::GridMap::addRectangleVerticesFromCellCenterTo(
                    outputGridMapAsMesh, tmpPosition, halfCellSize);
                addColors(BRIGHTBLUE, outputGridMapAsMesh.colors, 4);
            }
        }
    }
    outputGridMapAsMesh.header.frame_id = "ftm";
    double radius = 15;
    writeRemainingGridMapIntoGridMapMesh(
        radius, modifiedGridMap, outputGridMapAsMesh, currentOdometry);
    mapOfPublishers["pubGridMapAsMesh"].publish(outputGridMapAsMesh);
}

bool OperatorPerceptionModification::enterWasPressed() {
    bool returnBool { false };
    if ( percModStatus->getState() == (int) PerceptionModificationMode::OPERATOR_APPROVING ||
        percModStatus->getState() == (int) PerceptionModificationMode::OPERATOR_DRIVING ) {
        returnBool = true;
    }
    return returnBool;
}

void OperatorPerceptionModification::deleteThisCellFromModifiedGridMap(
const grid_map::Position& tmpPosition) {
    modifiedGridMap.atPosition(
        modifiedGridMap.getLayers().at(0), tmpPosition) = 0.0;
}

void OperatorPerceptionModification::deleteClicksAndPubIt() {
    listOfClicks.points.clear();
    publishListOfClicks();
}

void OperatorPerceptionModification::publishListOfClicks() {
    mapOfPublishers["pubMouseClicksForVisualization"].publish(listOfClicks);
    mapOfPublishers["pubMouseClicksForAreaVisualization"].publish(listOfClicks);
}

void OperatorPerceptionModification::writeRemainingGridMapIntoGridMapMesh(
            const double& radius, grid_map::GridMap& gridMap,
            tod_msgs::Mesh& gridMapMesh, const nav_msgs::Odometry& odometry) {
    double halfCellSize = gridMap.getResolution()/2;
    grid_map::Position currentVehiclePosition;
    currentVehiclePosition.x() = odometry.pose.pose.position.x;
    currentVehiclePosition.y() = odometry.pose.pose.position.y;
    grid_map::Position cellCenter;
    for ( grid_map::CircleIterator iterator(gridMap, currentVehiclePosition,
            radius); !iterator.isPastEnd(); ++iterator) {
        if ( gridMap.at(gridMap.getLayers().front(), *iterator) >=
                    OCCUPIED ) {
            TodHelper::GridMap::addRectangleIndicesTo(gridMapMesh);
            gridMap.getPosition(*iterator, cellCenter);
            TodHelper::GridMap::addRectangleVerticesFromCellCenterTo(
                gridMapMesh, cellCenter, halfCellSize);
            addColors(RED, gridMapMesh.colors, 4);
        }
    }
}

void OperatorPerceptionModification::addObjectToMesh(tod_msgs::Mesh& activeMesh,
        const std::vector<glm::vec3>& vectorToCorners, const unsigned int& index,
        const tod_msgs::Color& rgbColor) {
    addVertices(activeMesh, vectorToCorners);
    int numberOfVerticesPerObject{ 8 };
    addIndices(activeMesh.indices, index, numberOfVerticesPerObject);
    addColors(rgbColor, activeMesh.colors, numberOfVerticesPerObject);
}

void OperatorPerceptionModification::addVertices(tod_msgs::Mesh& activeMesh,
                const tod_msgs::ObjectData& currentObject) {
    std::vector<glm::vec3> vectorToCorners = TodHelper::ObjectList::getVectorsToCorners(
        currentObject);
    addVertices(activeMesh, vectorToCorners);
}

void OperatorPerceptionModification::addVertices(tod_msgs::Mesh& activeMesh,
            const std::vector<glm::vec3>& vectorToCorners) {
    geometry_msgs::Point tmpPoint;
    for ( auto& vector : vectorToCorners ) {
        tmpPoint.x = vector.x;
        tmpPoint.y = vector.y;
        tmpPoint.z = vector.z;
        activeMesh.vertices.push_back(tmpPoint);
    }
}

void OperatorPerceptionModification::addIndices(std::vector<unsigned int> &indices,
            const unsigned int index, const int numberOfVerticesPerObject) {
    unsigned int incre{ index*numberOfVerticesPerObject };
    std::vector<unsigned int> indicesOfAppendedObject {
        incre+0, incre+1, incre+2, incre+0, incre+2, incre+3,
        incre+0, incre+3, incre+4, incre+3, incre+4, incre+7,
        incre+4, incre+5, incre+6, incre+4, incre+6, incre+7,
        incre+1, incre+2, incre+5, incre+2, incre+5, incre+6,
        incre+0, incre+1, incre+5, incre+0, incre+5, incre+4,
        incre+3, incre+2, incre+6, incre+3, incre+6, incre+7
    };
    indices.insert(std::end(indices), std::begin(indicesOfAppendedObject),
                                    std::end(indicesOfAppendedObject));
}

void OperatorPerceptionModification::addColors(const tod_msgs::Color& rgbColor,
            std::vector<tod_msgs::Color>& colors, const int& numberOfColorsAdded) {
    for ( int iterator = 0; iterator != numberOfColorsAdded; ++iterator ) {
        colors.push_back(rgbColor);
    }
}

void OperatorPerceptionModification::callbackStatusMsg(const
            tod_msgs::Status& msg) {
    gotAllCallBacks.at(0) = 1;
    inputStatusMsg = msg;
    if ( msg.tod_status == tod_msgs::Status::TOD_STATUS_TELEOPERATION ) {
        statusIsTeleoperation = true;
    } else {
        statusIsTeleoperation = false;
    }
    if ( msg.tod_status == tod_msgs::Status::TOD_STATUS_IDLE ) {
        statusIsDisconnected = true;
    } else {
        statusIsDisconnected = false;
    }
}

void OperatorPerceptionModification::callbackObjectList(
    const tod_msgs::ObjectList& msg) {
    gotAllCallBacks.at(1) = 1;
    inputObjectList = msg;
}

void printSizeOfGridMapAndItsComponents(const grid_map_msgs::GridMap& inputGridMap) {
    ROS_ERROR("the grid map size is %lu consisting of"
    "info %lu "
    "layers %lu "
    "basic_layers %lu "
    "data %lu "
    "inner start index %lu "
    "outer start index %lu ",
    sizeof(inputGridMap),
    sizeof(inputGridMap.info),
    sizeof(inputGridMap.layers),
    sizeof(inputGridMap.basic_layers),
    sizeof(inputGridMap.data),
    sizeof(inputGridMap.outer_start_index),
    sizeof(inputGridMap.inner_start_index));
}

void OperatorPerceptionModification::callbackGridMap(const grid_map_msgs::GridMap& msg) {
    gotAllCallBacks.at(2) = 1;
    grid_map::GridMapRosConverter::fromMessage(msg, inputGridMap);
}

void OperatorPerceptionModification::callbackGridMapVisualization(const tod_msgs::Mesh& msg) {
    gotAllCallBacks.at(3) = 1;
    inputGridMapAsMesh = msg;
}

void OperatorPerceptionModification::callbackMouseClicks(
        const geometry_msgs::Point& msg) {
    gotAllCallBacks.at(4) = 1;
    if ( !statusIsTeleoperation || !controlModeIsChosen() ) {
        return;
    }
    float maximumAcceptedClickHeight{ 0.025 };
    if ( msg.z > maximumAcceptedClickHeight ) {
        ROS_ERROR("Click is not accepted in OperatorPerceptionModification because"
                    " click height = %5f, which is higher than accepted height of %5f",
                    msg.z, maximumAcceptedClickHeight);
        return;
    }
    if ( percModStatus->getState() > (int) PerceptionModificationMode::OPERATOR_PLANNING ) {
        ROS_ERROR("click ENTF to get back into drawing mode");
        return;
    }
    geometry_msgs::Point32 tmpPoint;
    tmpPoint.x = msg.x;
    tmpPoint.y = msg.y;
    tmpPoint.z = maximumAcceptedClickHeight;
    listOfClicks.points.push_back(tmpPoint);
    gotNewMouseClicks = true;
}

void OperatorPerceptionModification::callbackKeyPress(const
        tod_msgs::KeyPress& msg) {
    gotAllCallBacks.at(5) = 1;
    if ( !statusIsTeleoperation || !controlModeIsChosen() ) {
        return;
    }
    if ( (int) msg.key == (int) GLFW_KEY_DELETE ) {
        deleteClicksAndPubIt();
        percModStatus->resetState();
        tellVehicleToStop();
    }
    if ( (int) msg.key == (int) GLFW_KEY_ENTER ) {
        processClickOnEnter();
    }
    if ( (int) msg.key == (int) GLFW_KEY_KP_5 ) {
        handleVelocityIncrement(false);
    }
    if ( (int) msg.key == (int) GLFW_KEY_KP_8 ) {
        handleVelocityIncrement(true);
    }
}

void OperatorPerceptionModification::processClickOnEnter() {
    if ( listOfClicks.points.empty() ) {
        // nothing should happen, if nothing was clicked before
    } else {
        if ( percModStatus->getState() == PerceptionModificationMode::OPERATOR_PLANNING ) {
            percModStatus->increaseState();
            updateOperatorRequest();
            publishOperatorRequestToBeSentToVehicle();
        }
        if ( percModStatus->getState() == PerceptionModificationMode::OPERATOR_APPROVING ) {
            percModStatus->increaseState();
            updateOperatorApproval();
            publishOperatorApprovalToBeSentToVehicle();
        }
    }
}

void OperatorPerceptionModification::tellVehicleToStop() {
    updateOperatorRequest(); //listOfClicks.empty() -> vehicle reverts prev area
    publishOperatorRequestToBeSentToVehicle();
}

void OperatorPerceptionModification::handleVelocityIncrement(const bool positiveIncrement) {
    if ( percModStatus->getState() != (int) PerceptionModificationMode::OPERATOR_DRIVING ) {
        return; // only change in driving mode
    }
    std_msgs::Bool tmpBool;
    tmpBool.data = positiveIncrement;
    mapOfPublishers["operator_increment_velocity"].publish(tmpBool);
}

void OperatorPerceptionModification::callbackOdometry(const nav_msgs::Odometry& msg) {
    gotAllCallBacks.at(6) = 1;
    currentOdometry = msg;
}

bool OperatorPerceptionModification::controlModeIsChosen() {
    return controlModeChecker->modeIsChosen();
}

void OperatorPerceptionModification::updateOperatorRequest() {
    operatorRequest.header.stamp = ros::Time::now();
    operatorRequest.mouseClicks = listOfClicks;
    operatorRequest.RequestId = std::rand();
}

void OperatorPerceptionModification::publishOperatorRequestToBeSentToVehicle() {
    mapOfPublishers["operator_request"].publish(operatorRequest);
}

void OperatorPerceptionModification::callbackVehicleResponse(
const tod_perception_modification::PercModVehicleResponse& msg) {
    vehicleResponse = msg;
    if ( vehicleResponse.RequestId == operatorRequest.RequestId &&
    vehicleResponse.mouseClicksAreAccepted ) {
        if ( percModStatus->getState() ==
        PerceptionModificationMode::OPERATOR_SENT_MODIFICATION_REQUEST ) {
            percModStatus->increaseState();
        }
    }
}

void OperatorPerceptionModification::updateOperatorApproval() {
    operatorApproval.header.stamp = ros::Time::now();
    operatorApproval.operatorApproved = true;
    operatorApproval.RequestId = vehicleResponse.RequestId;
}

void OperatorPerceptionModification::publishOperatorApprovalToBeSentToVehicle() {
    mapOfPublishers["operator_approval"].publish(operatorApproval);
}
