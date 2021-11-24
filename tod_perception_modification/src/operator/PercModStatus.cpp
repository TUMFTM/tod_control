// Copyright 2020 Feiler

#include "PercModStatus.h"

PercModStatus::PercModStatus() :
state(PerceptionModificationMode::OPERATOR_PLANNING) {
}

int PercModStatus::getState() {
    return (int) state;
}

void PercModStatus::increaseState() {
    if ( state == PerceptionModificationMode::OPERATOR_DRIVING ) {
        return;
    }
    state = (PerceptionModificationMode) ((int) state + 1);
}

void PercModStatus::resetState() {
    state = PerceptionModificationMode::OPERATOR_PLANNING;
}
