// Vehicle Actuation Module main header
// Matt Young, 2021
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
    ebsReason = reason; \
    safetyState = VAM_SOFTEBS_ACTIVE; \
    updateSafetyFsm(); /* required to activate SoftEBS instantly */\
    return; \
} while (0);

// FIXME we can't do the commented out stuff in the above macro because _reason_ can be a variable or a string
// FIXME we would need to make a function called like activateSoftEbs or whatever
/*if (safetyState == VAM_SOFTEBS_ACTIVE) { \
    ROS_WARN("ATTENTION: An additional fault occurred during SoftEBS activation: %s", reason.c_str()); \
    return; \
} \*/

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
    /// Activates the software EBS - stop the vehicle using regen brake ASAP
    void activateSoftEbs(void);

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
    /*
        handle.subscribe("/sbg/status", 1, &VAM::insStatusCallback, this);
    handle.subscribe("/imu/odometry", 1, &VAM::odomCallback, this);
    handle.subscribe("/vehicle/safety", 1, &VAM::safetyCallback, this);
    handle.subscribe("/vehicle/cmd_vel", 1, &VAM::cmdVelCallback, this);
    */
   ros::Subscriber insStatusSub, imuOdometrySub, vehicleSafetySub, vehicleCmdVelSub;
};
