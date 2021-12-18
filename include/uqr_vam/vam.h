#pragma once
#include <ros/ros.h>
#include <sbg_driver/SbgStatus.h>
#include <sbg_driver/SbgEkfNav.h>
#include <sbg_driver/SbgEkfEuler.h>

enum VAMSafetyState {
    /** EBS is not active, car is operating normally */
    VAM_OK,
    /** EBS has just been requested, activate it */
    VAM_EBS_REQUESTED,
    /** EBS request has gone through, car is being stopped and is dangerous! */
    VAM_DANGER,
};

class VAM {
public:
    VAM(ros::NodeHandle &handle);
    ~VAM() = default;

    void loop(void);

private:
    void insStatusCallback(const sbg_driver::SbgStatus &status);
    void insEkfNavCallback(const sbg_driver::SbgEkfNav &nav);

    VAMSafetyState safetyState = VAM_OK;
    std::string ebsReason = "EBS OK";
};
