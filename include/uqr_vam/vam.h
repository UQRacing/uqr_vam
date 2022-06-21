/*
 * Vehicle Actuation Module main header
 * Copyright (c) 2022 Matt Young (UQ Racing Formula SAE Team)
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once
#include <ros/ros.h>
#include <sbg_driver/SbgStatus.h>
#include <sbg_driver/SbgEkfNav.h>
#include <sbg_driver/SbgEkfEuler.h>
#include <nav_msgs/Odometry.h>
#include "uqr_vam/timeout.h"
#include <uqr_msgs/CmdVel.h>
#include <uqr_msgs/Safety.h>

/// Activate SoftEBS and return
#define VAM_SOFTEBS_ACTIVATE(reason) do { \
    activateSoftEbsMacro(reason); \
    return; \
} while (0);

enum VAMSafetyState {
    /** SoftEBS is not active, car is operating normally */
    VAM_OK,
    /** SoftEBS has been activated, we must stop now and not continue */
    VAM_SOFTEBS_ACTIVE,
};

class VAM {
public:
    VAM(ros::NodeHandle &handle);
    ~VAM() = default;

    /// Updates safety FSM
    void updateSafetyFsm(void);
    /// Updates control signals to SmartMotor (steering) and WaveScultor (engine) - based on FSM state
    void updateControl(void);

private:
    /// Called every tick to actually check safety conditions (e.g. timeouts)
    void checkSafety(void);
    /// Configures the car hardware to implement the SoftEBS: activate regen brake and stop car instantly
    void startSoftEbs(void);
    /// Function used in VAM_SOFTEBS_ACTIVATE macro
    void activateSoftEbsMacro(std::string reason);

    // INS callbacks
    void insStatusCallback(const sbg_driver::SbgStatus &status);
    void odomCallback(const nav_msgs::Odometry &odometry);

    // UQR callbacks
    void safetyCallback(const uqr_msgs::Safety &safety); // callback to check if EBS was requested by a node
    void cmdVelCallback(const uqr_msgs::CmdVel &cmdVel); // callback to check CmdVel message is being updated frequently

    // flags
    VAMSafetyState safetyState = VAM_OK;
    std::string ebsReason = "SoftEBS has not been activated";

    // timeouts
    Timeout insTimeout;
    Timeout controlTimeout;

    // loaded from config
    double insTimeoutValue, insInitTimeoutValue, controlTimeoutValue, controlInitTimeoutValue;

    // publishers
    // TODO we will need some CAN stuff here for uqr_ccm

    // subscribers
   ros::Subscriber insStatusSub, imuOdometrySub, vehicleSafetySub, vehicleCmdVelSub;
};
