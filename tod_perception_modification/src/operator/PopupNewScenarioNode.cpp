// Copyright 2021 Feiler

#include "ros/ros.h"
#include <QApplication>

#include "PopupNewScenario.h"

int main(int argc, char **argv) {
    QApplication application(argc, argv);
    application.setQuitOnLastWindowClosed(false);
    ros::init(argc, argv, "PopupNewScenario");
    PopupNewScenario popUpNewScenario;
    return application.exec();
}
