//Copyright 2021 Feiler

#include "PopupNewScenario.h"
#include <thread>

PopupNewScenario::PopupNewScenario() {
    itsRosThread = std::thread(&PopupNewScenarioRosThread::runRosLoop,
        &popupNewScenarioRosThread);
    checkTimer = new QTimer;
    connect(checkTimer, &QTimer::timeout,
        this, &PopupNewScenario::closeApplicationIfNeeded);
    connect(checkTimer, &QTimer::timeout,
        this, &PopupNewScenario::showAPopupIfTriggered);
    checkTimer->start();
    msgBox = new QMessageBox;
    msgBox->setParent(0);
    msgBox->setStandardButtons(QMessageBox::Ok);
    msgBox->setDefaultButton(QMessageBox::Ok);
    msgBox->setIcon(QMessageBox::Warning);
    msgBox->setWindowTitle(tr("Help Request"));
    msgBox->setStyleSheet("QLabel{min-width: 700px; font-size: 18px;}");
    mqttClientToReceiveTriggeredTask =
        std::make_unique<tod_network::MqttClientTemplated<PopupNewScenario>>(
        "127.0.0.1", "PopupNewScenario");
    if ( !mqttClientToReceiveTriggeredTask->is_connected() ) {
        ROS_ERROR("Problems at localhost. Is mosquitto mqtt installed?\n");
    }
    mqttClientToReceiveTriggeredTask->subscribe("/TaskTrigger", 1,
        &PopupNewScenario::mqttCallback, this);
}

PopupNewScenario::~PopupNewScenario() {
    delete checkTimer;
}

void PopupNewScenario::closeApplicationIfNeeded() {
    if ( popupNewScenarioRosThread.rosWasShutDown() ) {
        itsRosThread.join();
        QApplication::quit();
    }
}

void PopupNewScenario::showAPopupIfTriggered() {
    if ( gotNewTask ) {
        gotNewTask = false;
        popupNewScenarioRosThread.publishThisTaskNumber(currentTaskNumber);
        std::string message;
        if ( currentTaskNumber == 1 ) {
            message = std::string("Fahre möglichst nahe an dem Würfel um die\n") +
            std::string("Linkskurve, ohne dabei den Würfel zu berühren.\n") +
            std::string("Wähle als Geschwindigkeit 7 km/h.");
        } else if (currentTaskNumber == 2) {
            message = std::string("Fahre möglichst nahe an den Würfeln vorbei,\n") +
            std::string("ohne dabei die Würfel zu berühren.\n") +
            std::string("Wähle als Geschwindigkeit 7 km/h.");
        } else if (currentTaskNumber == 3) {
            message = std::string("Fahre 15 km/h. Halte mit einer angenehmen \n") +
            std::string("Verzögerung exakt an der Stoplinie.\n");
        } else if (currentTaskNumber == 4) {
            message = std::string("Fahre mit maximal 7 km/h zwischen\nden Würfeln hindurch,\n") +
            std::string("ohne dabei die Würfel zu berühren");
        } else if (currentTaskNumber == 6) {
            message = std::string("Parke mit maximal 7 km/h quer zur Straße\n") +
            std::string("zwischen den Würfeln,\n") +
            std::string("ohne dabei die Würfel zu berühren und \n") +
            std::string("bleibe exakt an der Haltelinie stehen");
        } else {
            message = std::string("Bitte löse die Situation.");
        }
        msgBox->setText(tr(message.c_str()));
        msgBox->exec();
    }
}

void PopupNewScenario::mqttCallback(const mqtt::const_message_ptr msg) {
    ros::serialization::IStream stream((uint8_t*) msg->get_payload_str().c_str(),
        msg->get_payload().length());
    std_msgs::Int32 receivedInteger;
    ros::serialization::Serializer<std_msgs::Int32>::read(stream, receivedInteger);
    currentTaskNumber = receivedInteger.data;
    gotNewTask = true;
}
