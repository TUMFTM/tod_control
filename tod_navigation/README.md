# tod_navigation

This package parses a xml-route, sends/receives it and provides Mesh data for visualization in tod_visual.

## Dependencies

* ROS Packages: see `package.xml`
* Third Party:

    * Crushed Pixel

        ```bash
        git clone https://github.com/CrushedPixel/Polyline2D.git polyline
        cd polyline
        mkdir build
        cd build
        cmake ..
        sudo make install
        cd ../..
        sudo rm -rf polyline
        ```

This package asks the user, which route should be loaded.
The route needs to be stored in an xml file (xml/ExampleRoute.xml).
The route is than loaded and published as an nav_msgs/Path.

## Documentation of nodes

### VehicleRouteProviderNode

This node parses an xml-formatted route from the ROS parameter server. The location of the xml-route is specified in the `tod_navigation.launch` file.

Published Topics:

* /Vehicle/Navigation/route [nav_msgs::Path]

Subscriptions: No subscriptions

### OperatorRouteToMeshMsgNode

This node converts the provided path into a MeshMsg to be displayed in tod_visual.

Published Topics:

* /Operator/Visual/RouteAsMesh [tod_msgs::MeshMsg]

Subscriptions:

* /Vehicle/Navigation/route [nav_msgs::Path]

## Quick start

1. Create a xml-route:

    ```python2.7
    python2.7 py/bag_to_xml.py -i <input.bag> -o <output.xml> -t <Odometry topic>
    ```

    or use the provided `xml/ExampleRoute.xml`

2. Adapt `pathToRoute` in tod_navigation.launch to your appropriate xml-route path
3. Build and launch the package either via top-level launch files (operator/vehicle.launch) or by

    ```bash
    catkin build
    source devel/setup.zsh (or setup.bash)
    roslaunch tod_navigation tod_navigation.launch
    ```
