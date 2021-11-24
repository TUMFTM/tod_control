// Copyright 2020 Feiler

#pragma once

#include "tod_msgs/Mesh.h"
#include <vector>

namespace TodHelper {

class GridMap {
public:
    inline static void addRectangleIndicesTo(tod_msgs::Mesh& mesh) {
        std::vector<unsigned int> indices {
            (unsigned int) mesh.vertices.size() + 0,
            (unsigned int) mesh.vertices.size() + 1,
            (unsigned int) mesh.vertices.size() + 2,
            (unsigned int) mesh.vertices.size() + 2,
            (unsigned int) mesh.vertices.size() + 3,
            (unsigned int) mesh.vertices.size() + 0
        };
        mesh.indices.insert(std::end(mesh.indices),
                    std::begin(indices), std::end(indices));
    }

    //     ___ ___
    //    | 4   3 |
    //    |       |
    //    | 1   2 |
    //    |___ ___|   | x
    //            y __|
    // map is always aligned to x and y of grid_frame (= ftm in this case)
    // --> no iterative update on cell corner vectors needed.
    // it is always: +/- half cell size
    inline static void addRectangleVerticesFromCellCenterTo(
            tod_msgs::Mesh& mesh, const grid_map::Position& cellCenter,
            const double& halfCellSize) {
        grid_map::Position edge1;
        grid_map::Position edge2;
        grid_map::Position edge3;
        grid_map::Position edge4;
        edge1 = edge2 = edge3 = edge4 = cellCenter;
        edge1.x() -= halfCellSize;
        edge1.y() += halfCellSize;
        edge2.x() -= halfCellSize;
        edge2.y() -= halfCellSize;
        edge3.x() += halfCellSize;
        edge3.y() -= halfCellSize;
        edge4.x() += halfCellSize;
        edge4.y() += halfCellSize;
        geometry_msgs::Point point;
        point.x = edge1.x();
        point.y = edge1.y();
        mesh.vertices.push_back(point);
        point.x = edge2.x();
        point.y = edge2.y();
        mesh.vertices.push_back(point);
        point.x = edge3.x();
        point.y = edge3.y();
        mesh.vertices.push_back(point);
        point.x = edge4.x();
        point.y = edge4.y();
        mesh.vertices.push_back(point);
    }
};

} // namespace TodHelper
