// Vehicle Actuation Module main file
// Matt Young, 2021
#include "uqr_vam/vam.h"
#include "uqr_vam/timeout.h"
#include "uqr_vam/defines.h"

VAM::VAM(ros::NodeHandle &handle) {
    if (!handle.getParam("/vam/insTimeout", insTimeoutValue)) {
        ROS_ERROR("Failed to load INS timeout from config");
    }
    if (!handle.getParam("/vam/insFailedToInitTimeout", insInitTimeoutValue)) {
        ROS_ERROR("Failed to load insInitTimeoutValue from config");
    }
    if (!handle.getParam("/vam/controlTimeout", controlTimeoutValue)) {
        ROS_ERROR("Failed to load control timeout from config");
    }
    if (!handle.getParam("/vam/controlFailedToInitTimeout", controlInitTimeoutValue)) {
        ROS_ERROR("Failed to load control failed to init timeout from config");
    }

    handle.subscribe("/sbg/status", 1, &VAM::insStatusCallback, this);
    handle.subscribe("/imu/odometry", 1, &VAM::odomCallback, this);
    handle.subscribe("/vehicle/safety", 1, &VAM::safetyCallback, this);
    handle.subscribe("/vehicle/cmd_vel", 1, &VAM::cmdVelCallback, this);

    insTimeout = Timeout(insTimeoutValue, insInitTimeoutValue);
    controlTimeout = Timeout(controlTimeoutValue, controlInitTimeoutValue);
}

void VAM::activateSoftEbs(void) {
    //ROS_INFO("The SoftEBS would be activated here");
    // TODO activate SoftEBS
}

void VAM::insStatusCallback(const sbg_driver::SbgStatus &status) {
    // check general status
    if (!status.status_general.gps_power) {
        VAM_SOFTEBS_ACTIVATE("INS GPS power failure");
    } else if (!status.status_general.imu_power) {
        VAM_SOFTEBS_ACTIVATE("INS IMU power failure");
    } else if (!status.status_general.main_power) {
        VAM_SOFTEBS_ACTIVATE("INS main power failure");
    } else if (!status.status_general.settings) {
        VAM_SOFTEBS_ACTIVATE("INS settings load error");
    } else if (!status.status_general.temperature) {
        VAM_SOFTEBS_ACTIVATE("INS overheating!?");
    }

    // we don't call insTimeout.update() here because it is not actually useful information
}

void VAM::odomCallback(const nav_msgs::Odometry &odometry) {
    insTimeout.update();
    // we could also do some sanity checks here if we wanted to
}

void VAM::safetyCallback(const uqr_msgs::Safety &safety) {
    if (safety.softebs_activate) {
        VAM_SOFTEBS_ACTIVATE(safety.softebs_reason);
    }
}

void VAM::cmdVelCallback(const uqr_msgs::CmdVel &cmdVel) {
    controlTimeout.update();
    // also should probably do some sanity checks here
}

void VAM::checkSafety(void) {
    // INS checks
    if (!insTimeout.isAlive()) {
        VAM_SOFTEBS_ACTIVATE("INS timed out");
    }

    // Path planning and control checks
    if (!controlTimeout.isAlive()) {
        VAM_SOFTEBS_ACTIVATE("Control timed out");
    }
}

void VAM::updateSafetyFsm(void) {
    if (safetyState == VAM_OK) {
        // check for any safety problems, but operate normally
        checkSafety();
    } else if (safetyState == VAM_SOFTEBS_ACTIVE) {
        // activate SoftEBS and stay in this state (also keep sending activate EBS messages just in case)
        ROS_WARN_THROTTLE(1, "ATTENTION: SoftEBS has been activated! Reason: %s", ebsReason.c_str());
        activateSoftEbs();
    }
}

void VAM::updateControl(void) {
    if (safetyState == VAM_SOFTEBS_ACTIVE) {
        // EBS is active, just send stop car request and keep current steering angle
        // TODO send stop car message to wavesculptor
        return;
    }

    // TODO send normal control through
}

int main(int argc, char **argv){
    ROS_INFO("Vehicle Actuation Module v" VAM_VERSION);
    ros::init(argc, argv, "vam");
    ros::NodeHandle nodeHandle("vam");

    VAM vam = VAM(nodeHandle);

    // VAM runs at 25 Hz
    ros::Rate loopRate(25);
    while (ros::ok()) {
        vam.updateSafetyFsm();
        vam.updateControl();
        loopRate.sleep();
    }

    return EXIT_SUCCESS;
}