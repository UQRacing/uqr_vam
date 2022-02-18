// Timeout detector class for VAM
// Matt Young, 2022
#include "uqr_vam/timeout.h"

Timeout::Timeout(double expireTime, double failedInitTime) {
    this->expiry = expireTime;
    this->failedInit = failedInitTime;
    lastUpdateTime = ros::Time::now();
}

bool Timeout::isAlive(void) {
    double duration = (ros::Time::now() - lastUpdateTime).toSec();
    // if we died before, stay dead - even if the service may come back to life again
    if (dead) {
        return false;
    }
    // if the service has elapsed failedInitTime, it failed to initialise so we consider it to be dead
    if (duration >= failedInit) {
        dead = true;
        return false;
    }
    // if we're alive, and have expired the regular timeout, the service has died
    if (hasStarted && duration >= expiry) {
        dead = true;
        return false;
    }
    // the service is alive
    return true;
}

void Timeout::update(void) {
    hasStarted = true;
    lastUpdateTime = ros::Time::now();
}