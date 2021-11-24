// Copyright 2020 Feiler

#pragma once
#include "StateMachine.h"

enum PerceptionModificationMode {
    OPERATOR_PLANNING =                  0,
    OPERATOR_SENT_MODIFICATION_REQUEST = 1,
    OPERATOR_APPROVING =                 2,
    OPERATOR_DRIVING =                   3
};

class PercModStatus : public StateMachine {
    public:
        PercModStatus();
        void increaseState();
        void resetState();
        int getState();

    private:
        PerceptionModificationMode state;
};
