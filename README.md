# Anti-Satellite Interceptor Control
 A guidance and control system for an anti-satellite missile, coded in kOS for KSP.

[Demonstration Video Link](https://youtu.be/yCiPXL6n5Ro)

Ideally, a satellite interceptor would be launched as late as possible to ensure minimum time-to-kill. Therefore, the point of interception to be targeted lies on the plane that includes the launch site and is normal to the orbit of the target. For a zero inclination, zero eccentricity target with an interceptor launched from the equator, the point of interception is directly overhead the launch site. For cases where the target trajectory passes to either side of the launch site, the point is off to that side.

For our two-stage interceptor, both stages operate under a pure open loop control system. Guidance calculates the correct pitch rate to guarantee (approximately) the trajectory passes through the chosen intercept point, and then the vehicle flies with that parameter. When the payload separates from the second stage, it uses a reaction control system to closed loop refine the closest approach to be at the intercept point and occurring at the same time. We call this last step the terminal homing phase.
