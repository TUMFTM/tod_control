//Copyright 2021 Feiler

#pragma once
#include <QApplication>
#include <QTimer>
#include "PopupNewScenarioRosThread.h"
#include <thread>
#include <QMessageBox>
#include <string>
#include <memory>
#include "tod_network/mqtt_client_templated.h"

class PopupNewScenario  : public QObject  {
    Q_OBJECT

public:
    PopupNewScenario();
    ~PopupNewScenario();

private:
    PopupNewScenarioRosThread popupNewScenarioRosThread;
    std::thread itsRosThread;
    QTimer* checkTimer;
    QMessageBox* msgBox;
    std::unique_ptr<tod_network::MqttClientTemplated<PopupNewScenario>>
        mqttClientToReceiveTriggeredTask;
    
    int currentTaskNumber;
    bool gotNewTask { false };
    
    void closeApplicationIfNeeded();
    void showAPopupIfTriggered();
    void mqttCallback(const mqtt::const_message_ptr msg);
};
