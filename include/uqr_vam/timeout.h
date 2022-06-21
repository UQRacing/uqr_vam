/*
 * Timeout detector class for VAM
 * Copyright (c) 2022 Matt Young (UQ Racing Formula SAE Team)
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once
#include <stdint.h>
#include <ros/ros.h>

/// Class used to monitor if services are still alive
class Timeout {
public:
    /**
     * Creates a new timeout to monitor a service
     * @param expireTime time until this timeout will expire in seconds
     * @param failedInitTime time until this timeout is considered to have failed to initialise
     */
    Timeout(double expireTime, double failedInitTime);
    Timeout() = default;
    ~Timeout() = default;

    /**
     * Resets the timeout, indicating the service being monitored is alive.
     * The first call to this function signifies that the service has initialised and is sending useful data.
     */
    void update(void);
    
    /// @return true if the service is alive, otherwise false. Does not update the timeout at all.
    bool isAlive(void);

private:
    /// regular timeout, used after we start receiving useful data out of this service
    double expiry;
    /// timeout for if we failed to initialise (longer)
    double failedInit;
    /// true if we've received a useful message yet
    bool hasStarted = false;
    /// true if the service has died or failed to initialise
    bool dead = false;
    ros::Time lastUpdateTime;
};