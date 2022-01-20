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
    // if we died before, stay dead
    if (dead) {
        return false;
    }
    // if the service has elapsed failedInitTime, regardless of if it's alive or not, it's definitely dead
    if (duration >= failedInit) {
        dead = true;
        return false;
    }
    // if we're alive, and have expired the regular timeout, the service is dead
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