// Vehicle Actuation Module main file
// Matt Young, 2021
#include "uqr_vam/vam.h"
#include "uqr_vam/defines.h"

VAM::VAM(ros::NodeHandle &handle) {
    if (!handle.getParam("/vam/insTimeout", insTimeout)) {
        ROS_ERROR("Failed to load INS timeout from config");
    }
    if (!handle.getParam("/vam/insFailedToInitTimeout", insFailedToInitTimeout)) {
        ROS_ERROR("Failed to load insFailedToInitTimeout from config");
    }

    insStatusSub = handle.subscribe("/sbg/status", 1, &VAM::insStatusCallback, this);
    insEkfNavSub = handle.subscribe("/imu/odometry", 1, &VAM::odomCallback, this);
    
    lastOdomTime = ros::Time::now();
}

void VAM::activateEbs(void) {
    ROS_INFO("The EBS would be activated here");
    // TODO publish EBS active to dfmm (I assume using uqr_ccm)
}

void VAM::insStatusCallback(const sbg_driver::SbgStatus &status) {
    // check general status
    if (!status.status_general.gps_power) {
        VAM_EBS_ACTIVATE_R("INS GPS power failure");
    } else if (!status.status_general.imu_power) {
        VAM_EBS_ACTIVATE_R("INS IMU power failure");
    } else if (!status.status_general.main_power) {
        VAM_EBS_ACTIVATE_R("INS main power failure");
    } else if (!status.status_general.settings) {
        VAM_EBS_ACTIVATE_R("INS settings load error");
    } else if (!status.status_general.temperature) {
        VAM_EBS_ACTIVATE_R("INS overheating!?");
    }
}

void VAM::odomCallback(const nav_msgs::Odometry &odometry) {
    if (!firstOdomMsg) {
        // once we receive our first INS odometry message, start timeout detector
        ROS_INFO("Starting INS odometry timeout detector");
        firstOdomMsg = true;
    }
    lastOdomTime = ros::Time::now();

    // we could also do some sanity checks here if we wanted to
}

void VAM::checkSafety(void) {
    double insElapsed = (ros::Time::now() - lastOdomTime).toSec();

    // INS CHECKS
    // check if it's been ages since we initialised
    if (insElapsed >= insFailedToInitTimeout) {
        VAM_EBS_ACTIVATE_R("INS probably failed to initialise");
    }
    // otherwise check if we haven't received a useful INS message in a while, after receiving one before
    if (insElapsed >= insTimeout && firstOdomMsg) {
        VAM_EBS_ACTIVATE_R("INS odometry timeout");
    }

    // STEERING CHECKS
    // TODO
}

void VAM::updateSafetyFsm(void) {
    if (safetyState == VAM_OK) {
        // check for any safety problems, if there are any, we will activate the EBS
        checkSafety();
    } else if (safetyState == VAM_EBS_REQUESTED) {
        // activate EBS
        ROS_WARN("Activating EBS NOW! Reason: %s", ebsReason.c_str());
        activateEbs();
        safetyState = VAM_EBS_DONE;
    } else if (safetyState == VAM_EBS_DONE) {
        // just sit here while the EBS stops the car - keep sending activate EBS messages in case a message
        // was missed somehow
        ROS_WARN_THROTTLE(5, "EBS has been activated! Stand clear of vehicle!");
        activateEbs();
    }
}

void VAM::updateControl(void) {
    if (safetyState == VAM_EBS_DONE || safetyState == VAM_EBS_REQUESTED) {
        // EBS is active, just send stop car request and stop steering
        // TODO send stop car message to wavesculptor
        return;
    }
}

int main(int argc, char **argv){
    ROS_INFO("Vehicle Actuation Module v" VAM_VERSION);
    ros::init(argc, argv, "vam");
    ros::NodeHandle nodeHandle("vam");

    VAM vam(nodeHandle);
    ROS_INFO("Waiting for first messages to arrive...");

    // VAM runs at 10 Hz
    ros::Rate loopRate(10);
    while (ros::ok()) {
        vam.updateSafetyFsm();
        vam.updateControl();
        loopRate.sleep();
    }

    return EXIT_SUCCESS;
}