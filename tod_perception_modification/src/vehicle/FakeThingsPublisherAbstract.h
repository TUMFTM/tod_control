// Copyright 2021 Feiler

#pragma once
#include <string>

enum Mode {
    CREATE          = 0,
    APPEND          = 1
};

class FakeThingsPublisherAbstract {
public:
    virtual ~FakeThingsPublisherAbstract() = default;
    virtual void initFromParamServer(const std::string& nodeName) = 0;
    virtual bool isCreateMode() = 0;
    virtual void create() = 0;
    virtual void publish() = 0;
};
