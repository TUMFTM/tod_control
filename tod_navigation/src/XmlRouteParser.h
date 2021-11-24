// Copyright 2021 Feiler

#include <string>
#include "tinyxml2.h"
#include "nav_msgs/Path.h"

class XmlRouteParser {
public:
    XmlRouteParser() = default;
    void setPathToXml(const std::string& path);
    void parse();
    nav_msgs::Path getRoute();

private:
    std::string _path;
    tinyxml2::XMLDocument _xmlDoc;
    nav_msgs::Path _route;

    void loadFile();
    void throwIfRouteElementIsNotInXml(tinyxml2::XMLElement* route);
    void addAllPointsToNavMsgsPath(tinyxml2::XMLElement* point);
};
