// Vehicle Actuation Module main header
// Matt Young, 2021
#pragma once
#include <ros/ros.h>
#include <sbg_driver/SbgStatus.h>
#include <sbg_driver/SbgEkfNav.h>
#include <sbg_driver/SbgEkfEuler.h>
#include <nav_msgs/Odometry.h>

/// Activate EBS and return
#define VAM_EBS_ACTIVATE_R(reason) do { \
    ebsReason = reason; \
    safetyState = VAM_EBS_REQUESTED; \
    updateSafety(); /* required to activate EBS instantly */\
    return; \
} while (0);

/// Activate EBS but do not return
#define VAM_EBS_ACTIVATE(reason) do { \
    ebsReason = reason; \
    safetyState = VAM_EBS_REQUESTED; \
    updateSafety(); /* required to activate EBS instantly */\
} while (0);

enum VAMSafetyState {
    /** EBS is not active, car is operating normally */
    VAM_OK,
    /** EBS has just been requested, we are activating it */
    VAM_EBS_REQUESTED,
    /** EBS request has gone through, car is being stopped and may be dangerous */
    VAM_EBS_DONE,
};

class VAM {
public:
    VAM(ros::NodeHandle &handle);
    ~VAM() = default;

    /// Updates safety FSM
    void updateSafety(void);
    /// Updates control signals
    void updateControl(void);

private:
    /// Called every tick to actually check safety conditions (e.g. timeouts)
    void checkSafety(void);
    /// Activates the EBS hardware
    void activateEbs(void);

    // INS callbacks
    void insStatusCallback(const sbg_driver::SbgStatus &status);
    void odomCallback(const nav_msgs::Odometry &odometry);

    // UQR callbacks
    // TODO need uqr_msgs
    void ebsCallback(); // callback to activate EBS directly
    void steeringCallback(); // callback to check steering angles are being updated frequently

    // flags
    VAMSafetyState safetyState = VAM_OK;
    std::string ebsReason = "EBS has not been activated yet";
    
    /// true if first useful INS message has been received yet
    bool firstOdomMsg = false;
    /// last useful INS message time
    ros::Time lastOdomTime;

    // config
    double insTimeout;

    // subscribers
    ros::Subscriber insStatusSub, insEkfNavSub;

    // publishers
    // ...
};
