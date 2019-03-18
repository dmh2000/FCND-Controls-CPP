# C++ Controller Solution
## David Howard
##  dmh2000@gmail.com

## Body Rate Controller Implementation

## Roll-Pitch Controller Implementation

## Altitude Controller Implementation

## Lateral Position Controller Implementation

## Yaw Controller Implementation

## Motor Command Implementation

## Flight Evaluation

### Scenario 1 - Intro
The goal of this scenario was to modify the mass value of the vehicle
so that it held altitude. The original value was .4 kilograms, and
the value .495 passed the test. 

The file config/QuadControlParams-S1.txt has the gains that passed
the tests in this scenario.

### Scenario 2 - Attitude Control
The goal of this scenario was to implement the GenerateMotorCommands,
BodyRateControl and RollPitchControl functions. Following that,
it was required to tune the kpPQR and kpBank values so the vehicle roll was
stabilized.

This was the second most difficult scenario to implement correctly. The
BodyRateControl function was easy, as it followed exactly from the
lesson. The RollPitchControl was also straighforward, as it
followed the lessons. The difficult part was the GenerateMotorCommnands, 
as it deviated a bit from the projects in the lesson. It was
clear that unless this function was implemented properly, nothing
would work. It wasn't clear to me from the lessons that the motor commands were a relative simple
modification of the  set_propeller_angular_velocities function
from the exercises. Once I resolved the differences in implementation
the scenario began to pass. I required several iterations of 

Tuning of kpPQR was straigtforward. I simply doubled the initial value (23) and twiddled it a little
and the test passed. The kpPQR gains that worked were relatively low compared to what was needed later. 
The test passed with kpPQR = 42,42,15. This threw me my tuning for scenario 3.

The file config/QuadControlParams-S2 has the gains that passed the test in this
scenario.

The gif recorder cut off the final frames that showed the success green bars but 

![scenario 2](scenario-2.png)
![scenario 2](scenario-2.gif)


### Scenario 3 - Position Control
The goal of this scenario was to implement the  AltitudeControl, LateralPositionControl
and YawControl functions. Then it was required to tune the kpPosXY, kpPosZ, kpVelXY, and
kpVelZ parameters so that the 3 quads in the test scenario arrived at the respective
target positions within the error limits. 

The altitude control followed the exercise on a feedfoward controller along with the lesson on PID control for
adding the integrator value. The lateral position controller and yaw controller were also straighforward. No constraints
were implemnted yet. 

Tuning of the parameters took several hours to pass the test. 

The file config/QuadControlParams-S3.txt has the gains that passed the test in this scenario.

![scenario 2](scenario-3.png)
![scenario 3](scenario-3.gif)

### Scenario 4 - Non-Idealities
The goal of this scenario was to tune the existing parameters to have the 3 quads fly to their respective
target position within the error and time limits. 

No changes to the implementations were required at this point. Quad 2, the 'normal' quad worked as is
with the existing parameters. Quad 1

![scenario 4](scenario-4.png)
![scenario 4](scenario-4.gif)


### Scenario 5 - TrajectoryFollowing

![scenario 5](scenario-5.png)
![scenario 5](scenario-5.gif)