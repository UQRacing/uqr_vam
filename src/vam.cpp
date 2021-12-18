#include "uqr_vam/vam.h"

int main(int argc, char **argv){
    ROS_INFO("Vehicle Actuation Module vTODO - Matt Young, 2021");
    ros::init(argc, argv, "vam");
    ros::NodeHandle nodeHandle("vam");

    //BrakeTestController brakeTestController(nodeHandle);

    // update safety loop at 10 Hz (http://wiki.ros.org/roscpp/Overview/Time)
    ros::Rate loopRate(10);
    while (ros::ok()) {
        //brakeTestController.controlLoop();
        loopRate.sleep();
    }

    return EXIT_SUCCESS;
}