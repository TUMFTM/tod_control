// Copyright 2020 Feiler

#include <gtest/gtest.h>
#include "tod_msgs/ObjectData.h"
#include "glm/glm.hpp"
#include "ObjectList.h"
#include <cmath>

static double pi{3.14159265359};

class test_getVectorsToCorners : public ::testing::Test {
    protected:

};

TEST_F(test_getVectorsToCorners, CenterPoint) {
    tod_msgs::ObjectData object;
    object.dimZ = 1.0;
    object.dimX = 1.0;
    object.dimY = 1.0;
    object.yawAngle = 0.0;
    object.distCenterX = 0.0;
    object.distCenterY = 0.0;
    std::vector<glm::vec3> vectorToCorners =
                    TodHelper::ObjectList::getVectorsToCorners(object);
    glm::vec3 rightRearLowCorner = vectorToCorners.at(0);
    EXPECT_EQ(rightRearLowCorner.x, -0.5);
    EXPECT_EQ(rightRearLowCorner.y, -0.5);
    EXPECT_EQ(rightRearLowCorner.z, 0.0);

    glm::vec3 leftRearLowCorner = vectorToCorners.at(1);
    EXPECT_EQ(leftRearLowCorner.x, -0.5);
    EXPECT_EQ(leftRearLowCorner.y, 0.5);
    EXPECT_EQ(leftRearLowCorner.z, 0.0);
}

TEST_F(test_getVectorsToCorners, CenterPointWithTranslation) {
    tod_msgs::ObjectData object;
    object.dimZ = 1.0;
    object.dimX = 1.0;
    object.dimY = 1.0;
    object.yawAngle = (float)pi/4;
    object.distCenterX = 2.0;
    object.distCenterY = 2.0;
    std::vector<glm::vec3> vectorToCorners =
                    TodHelper::ObjectList::getVectorsToCorners(object);
    glm::vec3 leftRearLowCorner = vectorToCorners.at(1);
    EXPECT_FLOAT_EQ(leftRearLowCorner.x, 2.0 - std::sqrt(2)/2);
    EXPECT_FLOAT_EQ(leftRearLowCorner.y, 2.0);
    EXPECT_FLOAT_EQ(leftRearLowCorner.z, 0.0);
}

TEST(test_createVectorOfObjectMeshes, VectorIsGivenBack) {
    tod_msgs::ObjectData exampleObject;
    exampleObject.dimZ = exampleObject.dimX = exampleObject.dimY = 1.0;
    tod_msgs::ObjectList exampleList;
    exampleList.objectList.push_back(exampleObject);
    exampleList.objectList.push_back(exampleObject);

    std::vector<std::vector<glm::vec3>> result = TodHelper::ObjectList::createVectorOfObjectMeshes(exampleList);

    ASSERT_EQ(result.size(), 2);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
