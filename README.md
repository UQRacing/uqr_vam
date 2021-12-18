# VAM
This is the Vehicle Actuation Module (VAM) for the AV. The VAM manages car-specific integration and is the
only part of the stack that knows about car-specific settings. It also manages safety and the EBS.

Maintainer: Matt Young (and probably Caleb Aitken)

## Vehicle integration
The VAM will turn actuation requests like target velocity and steering wheel angle into actual actuation
for a given car, and pass this further down the stack (i.e. to the smart motor controller or CAN).

## Safety
Very importantly, the VAM also manages the majority of safety including activating the EBS. The VAM listens
to a bunch of upstream topics and runs a heartbeat on each of them (e.g. if no control input is received,
the car will EBS). It also listens to the EBS topic itself and will of course EBS if that is received.

## Building and running
`catkin build`, `roslaunch uqr_vam vam.launch`