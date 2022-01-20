// Timeout detector class for VAM
// Matt Young, 2022
#pragma once
#include <stdint.h>
#include <ros/ros.h>

/// Class used to monitor if services are still alive
class Timeout {
public:
    /// @param expireTime time until this timeout will expire in seconds
    /// @param failedInitTime time until this timeout is considered to have failed to initialise
    Timeout(double expireTime, double failedInitTime);
    Timeout() = default;
    ~Timeout() = default;

    /// Resets the timeout, indicating the service being monitored is alive
    void update(void);
    
    /// @return true if the service is alive, otherwise false. Does not update the timeout at all.
    bool isAlive(void);

private:
    /// regular timeout for normal operation
    double expiry;
    /// timeout for if we failed to initialise (longer)
    double failedInit;
    /// true if we've received a useful message yet
    bool hasStarted = false;
    /// true if the service has died at least once
    bool dead = false;
    ros::Time lastUpdateTime;
};