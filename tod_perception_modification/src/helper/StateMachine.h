// Copyright 2021 Feiler

#pragma once
class StateMachine {
public:
    StateMachine() = default;
    virtual ~StateMachine() = default;
    virtual void increaseState() = 0;
    virtual void resetState() = 0;
    virtual int getState() = 0;
};
