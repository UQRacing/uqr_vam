# VAM
This is the Vehicle Actuation Module (VAM) for the AV. The VAM manages car-specific integration including
conversion of higher-level ROS commands into hardware-specific messages that will be sent to the vehicle's
CAN network using the CCM node. 

The VAM also manages the vehicle's safety by detecting faults and timeouts in all other nodes on the car.

Author: Matt Young (m.young2@uqconnect.edu.au)

## Vehicle integration
The VAM will turn actuation requests like target velocity and steering wheel angle into actual actuation
for a given car, and pass this further down the stack (i.e. to the smart motor controller and CAN).

## Safety
The VAM also manages the software side of the driverless vehicle's safety. While the VAM itself will currently
never activate the physical EBS (emergency brake system), it can instead activate a "SoftEBS" which
will cause the car to stop relatively suddenly using the regen brake.

The following is a full list of conditions that will cause the VAM to activate the SoftEBS:

**General**

- It is requested by a node publishing to the SoftEBS topic

**INS**

- The INS general status becomes non-OK (e.g. it's overheating)
- The INS odometry times out (no message after init and a delta of a few seconds)
- The INS failed to init (no message after many seconds)

**Path planning/control**

- The vehicle controller times out
- The vehicle controller failed to init

## Building and running
`catkin build`, `roslaunch uqr_vam vam.launch`

## Licence
Mozilla Public License v2.0