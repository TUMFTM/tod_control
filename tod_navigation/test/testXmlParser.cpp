// Copyright 2021 Feiler

#include "gtest/gtest.h"
#include "XmlRouteParser.h"
#include "nav_msgs/Path.h"
#include <unistd.h>         // readlink
#include <linux/limits.h>   // PATH_MAX
#include <string>
#include <ros/package.h>

class ParseRoute : public ::testing::Test {
    protected:
        XmlRouteParser xmlRouteParser;
        std::string pathToRoute;
        std::string pathToWrongFormattedRoute;
        void SetUp() override {
            std::string pathToTestFolder = ros::package::getPath("tod_navigation") + "/test";
            pathToRoute = pathToTestFolder + "/TestRoute.xml";
            pathToWrongFormattedRoute = pathToTestFolder + "/WrongFormat.xml";
        }
};

TEST_F(ParseRoute, throwBecauseNoCorrectPathIsProvided) {
    std::string emptyPath = "";
    xmlRouteParser.setPathToXml(emptyPath);

    EXPECT_THROW(xmlRouteParser.parse(), std::exception);
}

TEST_F(ParseRoute, noThrowBecauseFileExists) {
    xmlRouteParser.setPathToXml(pathToRoute);
    
    EXPECT_NO_THROW(xmlRouteParser.parse());
}

TEST_F(ParseRoute, readXPositionsCorrectly) {
    xmlRouteParser.setPathToXml(pathToRoute);
    xmlRouteParser.parse();
    
    nav_msgs::Path pathFromXml = xmlRouteParser.getRoute();
    ASSERT_EQ(0.0, pathFromXml.poses.at(0).pose.position.x);
    ASSERT_EQ(1.0, pathFromXml.poses.at(1).pose.position.x);
    ASSERT_EQ(2.0, pathFromXml.poses.at(2).pose.position.x);
}

TEST_F(ParseRoute, readYPositionsCorrectly) {
    xmlRouteParser.setPathToXml(pathToRoute);
    xmlRouteParser.parse();
    
    nav_msgs::Path pathFromXml = xmlRouteParser.getRoute();
    ASSERT_EQ(0.0, pathFromXml.poses.at(0).pose.position.y);
    ASSERT_EQ(-1.0, pathFromXml.poses.at(1).pose.position.y);
    ASSERT_EQ(-2.0, pathFromXml.poses.at(2).pose.position.y);
}

TEST_F(ParseRoute, wrongFormattedXml) {
    xmlRouteParser.setPathToXml(pathToWrongFormattedRoute);
    
    EXPECT_THROW(xmlRouteParser.parse(), std::exception);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
