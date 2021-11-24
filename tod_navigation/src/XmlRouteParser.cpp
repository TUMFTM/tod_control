// Copyright 2021 Feiler

#include "XmlRouteParser.h"
#include <stdexcept>
#include "geometry_msgs/PoseStamped.h"

void XmlRouteParser::setPathToXml(const std::string& path) {
    _path = path;
}

void XmlRouteParser::parse() {
    loadFile();
    // import data
    tinyxml2::XMLElement* route = _xmlDoc.FirstChildElement("Route");
    throwIfRouteElementIsNotInXml(route);
    tinyxml2::XMLElement* point = route->FirstChildElement("Point");
    if ( point == nullptr ) { // no information provided
        return;
    }
    _route.poses.clear();
    addAllPointsToNavMsgsPath(point);
}

void XmlRouteParser::loadFile() {
    tinyxml2::XMLError success = _xmlDoc.LoadFile(_path.c_str());
    if ( success != tinyxml2::XMLError::XML_SUCCESS ) {
        throw std::invalid_argument("XMLRouteParser: file could not be loaded");
    }
}

void XmlRouteParser::throwIfRouteElementIsNotInXml(tinyxml2::XMLElement* route) {
    if ( route == nullptr ) {
        throw std::invalid_argument(
            "XMLRouteParser: Route does not have correct format");
    }
}

void XmlRouteParser::addAllPointsToNavMsgsPath(tinyxml2::XMLElement* point) {
    geometry_msgs::PoseStamped tmpPose;
    for ( ; point != nullptr; point = point->NextSiblingElement() ) {
        point->QueryDoubleAttribute("x", &tmpPose.pose.position.x);
        point->QueryDoubleAttribute("y", &tmpPose.pose.position.y);
        _route.poses.push_back(tmpPose);
    }
}

nav_msgs::Path XmlRouteParser::getRoute() {
    return _route;
}
