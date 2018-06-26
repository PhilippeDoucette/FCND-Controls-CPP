# FCND-Controls-CPP project Submission #

Submission for the FCND-Controls-CPP project for Philippe Doucette.

### Files Modified and Added ###

The following files were modified:

`src/QuadControl.cpp`
 Primary source code for PID controller
 
`src/QuadControl.h`
 Minor modifications, adding variable declarations used in debug and testing code

`config/QuadControlParams.txt`
 Tuned parameters for PID controller
 
`config/6_Vertical.txt` and `7_Horizontal.txt`
 Additional Scenarios used to fine tune the controller parameters (Try them!)

`config/Scenarios.txt`
 List of scenarios that populates the right-click popup menu.  6 and 7 were added.
 
### The Code ###

For the project, the majority of my code modifications are in `QuadControl.cpp`.  I added a few variables to `QuadControl.h` that I used for debugging and tuning.

I tuned `QuadControlParams.txt` while writing the controller code and running scenarios.

I wrote two additional scenarios to help me tune the PID controller: 
 `6_Vertical.txt` flies the quads straight up and down (by modifying the start/end trajectories).  I could then examine the overshoot and tune the kpVelZ parameter.  
 `7_Horizontal.txt` is really just a drastic version of `4_Nonidealities` that I could modify for my own needs without polluting the pass/fail scenario.


### The Simulator ###

The `config/Scenarios.txt` was modified to add two additional Scenarios used to fine tune the controller parameters.  I found it difficult to tune the vertical parameters with the scenarios supplied.

### Testing it Out (scenario 1) ###

The thrusts are simply being set to:

```
QuadControlParams.Mass * 9.81 / 4
```

I tuned the `Mass = 0.5` parameter in `QuadControlParams.txt` to make the vehicle more or less stay in the same spot.

With the proper mass, the simulation looked like this:

<p align="center">
<img src="animations/scenario1.jpg" width="500"/>
</p>

This code was later commented out and replaced with controller code.

### Body rate and roll/pitch control (scenario 2) ###

Implemented the body rate and roll / pitch control. 


**Implemented body rate control**

 - implemented the code in the function `GenerateMotorCommands()`
   
    VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)

    float l = L / (float)sqrt(2);

    float tau_x = momentCmd.x / l;
    float tau_y = momentCmd.y / l;
    float tau_z = momentCmd.z / -kappa;

    float thrust_1 = (collThrustCmd + tau_x + tau_y + tau_z) / 4; // front left
    float thrust_2 = (collThrustCmd - tau_x + tau_y - tau_z) / 4; // front right
    float thrust_3 = (collThrustCmd + tau_x - tau_y - tau_z) / 4; // rear left These last two may be backwards
    float thrust_4 = (collThrustCmd - tau_x - tau_y + tau_z) / 4; // rear right

    cmd.desiredThrustsN[0] = thrust_1; // front left
    cmd.desiredThrustsN[1] = thrust_2; // front right
    cmd.desiredThrustsN[2] = thrust_3; // rear left
    cmd.desiredThrustsN[3] = thrust_4; // rear right

    //cmd.desiredThrustsN[0] = mass * 9.81f / 4.f; // front left
    //cmd.desiredThrustsN[1] = mass * 9.81f / 4.f; // front right
    //cmd.desiredThrustsN[2] = mass * 9.81f / 4.f; // rear left
    //cmd.desiredThrustsN[3] = mass * 9.81f / 4.f; // rear right
 
 
 - implement the code in the function `BodyRateControl()`
 
 
 
 - Tuned `kpPQR` in `QuadControlParams.txt` to get the vehicle to stop spinning quickly but not overshoot



**Implement roll / pitch control**

 - implemented the code in the function `RollPitchControl()`
 - Tuned `kpBank` in `QuadControlParams.txt` to minimize settling time but avoid too much overshoot

Successful, as the quad leveled itself, though it’ll still be flying away slowly since we’re not controlling velocity/position!  The vehicle angle (Roll) gets controlled to 0.

<p align="center">
<img src="animations/scenario2.jpg" width="500"/>
</p>


### Position/velocity and yaw angle control (scenario 3) ###

Next, you will implement the position, altitude and yaw control for your quad.  For the simulation, you will use `Scenario 3`.  This will create 2 identical quads, one offset from its target point (but initialized with yaw = 0) and second offset from target point but yaw = 45 degrees.

 - implement the code in the function `LateralPositionControl()`
 - implement the code in the function `AltitudeControl()`
 - tune parameters `kpPosZ` and `kpPosZ`
 - tune parameters `kpVelXY` and `kpVelZ`

If successful, the quads should be going to their destination points and tracking error should be going down (as shown below). However, one quad remains rotated in yaw.

 - implement the code in the function `YawControl()`
 - tune parameters `kpYaw` and the 3rd (z) component of `kpPQR`

Tune position control for settling time. Don’t try to tune yaw control too tightly, as yaw control requires a lot of control authority from a quadcopter and can really affect other degrees of freedom.  This is why you often see quadcopters with tilted motors, better yaw authority!

<p align="center">
<img src="animations/scenario3.gif" width="500"/>
</p>

**Hint:**  For a second order system, such as the one for this quadcopter, the velocity gain (`kpVelXY` and `kpVelZ`) should be at least ~3-4 times greater than the respective position gain (`kpPosXY` and `kpPosZ`).

### Non-idealities and robustness (scenario 4) ###

In this part, we will explore some of the non-idealities and robustness of a controller.  For this simulation, we will use `Scenario 4`.  This is a configuration with 3 quads that are all are trying to move one meter forward.  However, this time, these quads are all a bit different:
 - The green quad has its center of mass shifted back
 - The orange vehicle is an ideal quad
 - The red vehicle is heavier than usual

1. Run your controller & parameter set from Step 3.  Do all the quads seem to be moving OK?  If not, try to tweak the controller parameters to work for all 3 (tip: relax the controller).

2. Edit `AltitudeControl()` to add basic integral control to help with the different-mass vehicle.

3. Tune the integral control, and other control parameters until all the quads successfully move properly.  Your drones' motion should look like this:

<p align="center">
<img src="animations/scenario4.gif" width="500"/>
</p>


### Extra Challenge 1 (Optional) ###

You will notice that initially these two trajectories are the same. Let's work on improving some performance of the trajectory itself.

1. Inspect the python script `traj/MakePeriodicTrajectory.py`.  Can you figure out a way to generate a trajectory that has velocity (not just position) information?

2. Generate a new `FigureEightFF.txt` that has velocity terms
Did the velocity-specified trajectory make a difference? Why?

With the two different trajectories, your drones' motions should look like this:

<p align="center">
<img src="animations/scenario5.jpg" width="500"/>
</p>


### Extra Challenge 2 (Optional) ###

For flying a trajectory, is there a way to provide even more information for even better tracking?

How about trying to fly this trajectory as quickly as possible (but within following threshold)!


### Performance Metrics ###

All performance metrics were accomplished:

 - scenario 2
   - roll should less than 0.025 radian of nominal for 0.75 seconds (3/4 of the duration of the loop)
   - roll rate should less than 2.5 radian/sec for 0.75 seconds

 - scenario 3
   - X position of both drones should be within 0.1 meters of the target for at least 1.25 seconds
   - Quad2 yaw should be within 0.1 of the target for at least 1 second


 - scenario 4
   - position error for all 3 quads should be less than 0.1 meters for at least 1.5 seconds

 - scenario 5
   - position error of the quad should be less than 0.25 meters for at least 3 seconds
