// Vehicle Actuation Module main file
// Matt Young, 2021
#include "uqr_vam/vam.h"
#include "uqr_vam/defines.h"

VAM::VAM(ros::NodeHandle &handle) {
    if (!handle.getParam("/vam/insTimeout", insTimeout)) {
        ROS_ERROR("Failed to load INS timeout from config");
    }

    insStatusSub = handle.subscribe("/sbg/status", 1, &VAM::insStatusCallback, this);
    insEkfNavSub = handle.subscribe("/imu/odometry", 1, &VAM::odomCallback, this);
    
    lastOdomTime = ros::Time::now();
}

void VAM::activateEbs(void) {
    ROS_INFO("The EBS would be activated here");
    // TODO work out what requests to send to hardware to activate EBS
}

void VAM::insStatusCallback(const sbg_driver::SbgStatus &status) {
    // check general status
    if (!status.status_general.gps_power) {
        VAM_EBS_ACTIVATE_R("GPS power failure");
    } else if (!status.status_general.imu_power) {
        VAM_EBS_ACTIVATE_R("IMU power failure");
    } else if (!status.status_general.main_power) {
        VAM_EBS_ACTIVATE_R("Main power failure");
    } else if (!status.status_general.settings) {
        VAM_EBS_ACTIVATE_R("Settings load error");
    } else if (!status.status_general.temperature) {
        VAM_EBS_ACTIVATE_R("INS overheating!");
    }
}

void VAM::odomCallback(const nav_msgs::Odometry &odometry) {
    if (!firstOdomMsg) {
        ROS_INFO("First INS odometry message received");
        firstOdomMsg = true;
    }
    lastOdomTime = ros::Time::now();
}

void VAM::checkSafety(void) {
    // INS CHECKS
    // check if we've received an INS message
    double insElapsed = (ros::Time::now() - lastOdomTime).toSec();
    if (insElapsed >= insTimeout && firstOdomMsg) {
        VAM_EBS_ACTIVATE_R("INS odometry timeout");
    }
}

void VAM::updateSafety(void) {
    if (safetyState == VAM_OK) {
        // check for any safety problems, if there are any, we will activate the EBS
        ROS_INFO_THROTTLE(30, "The vehicle is operating normally");
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
        // EBS is active, send stop car request and stop steering
        // TODO
        return;
    }
}

int main(int argc, char **argv){
    ROS_INFO("Vehicle Actuation Module v" VAM_VERSION);
    ros::init(argc, argv, "vam");
    ros::NodeHandle nodeHandle("vam");

    VAM vam(nodeHandle);

    // VAM runs at 10 Hz (http://wiki.ros.org/roscpp/Overview/Time)
    ros::Rate loopRate(10);
    while (ros::ok()) {
        vam.updateSafety();
        vam.updateControl();
        loopRate.sleep();
    }

    return EXIT_SUCCESS;
}