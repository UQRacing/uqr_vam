# VAM
This is the Vehicle Actuation Module (VAM) for the AV. The VAM manages car-specific integration and is the
only part of the stack that knows about car-specific settings. It also manages safety and the EBS.

Maintainer: Matt Young (and probably Caleb Aitken)

## Vehicle integration
The VAM will turn actuation requests like target velocity and steering wheel angle into actual actuation
for a given car, and pass this further down the stack (i.e. to the smart motor controller or CAN).

## Safety
Very importantly, the VAM also manages the majority of safety including activating the EBS (emergency break).
The following is a full list of conditions that will cause the VAM to activate the EBS:

- It is requested by a node publishing to the EBS topic
- The INS general status becomes non-OK (e.g. it's overheating)
- (WIP) A useful INS message is not received

## Building and running
`catkin build`, `roslaunch uqr_vam vam.launch`