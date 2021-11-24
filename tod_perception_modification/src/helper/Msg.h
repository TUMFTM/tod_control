// Copyright 2020 Feiler

#pragma once

#include "tod_msgs/ObjectList.h"
#include "tod_msgs/Mesh.h"

namespace TodHelper {

class Msg {
public:
    inline static void clearObjectListMsg(tod_msgs::ObjectList& objectList) {
        objectList.objectList.clear();
    }

    inline static void clearMeshMsg(tod_msgs::Mesh& mesh) {
        mesh.colors.clear();
        mesh.indices.clear();
        mesh.vertices.clear();
    }
};

} // namespace TodHelper
