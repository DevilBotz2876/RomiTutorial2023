## Control Systems
### Feedforward
Although a **gross over-simplification**, feedforward control models the drivetrain and tries to **pre**-calculate the motor voltages needed to:

1. Overcome the *static* friction required to get the robot moving from a stationary position.  *E.g. A heavier robot will require a higher voltage to start moving than a heavier robot.* This is the `Ks` term.
1. Overcome any change in friction that that is *proportional to velocity*.  *E.g. An aerodynamic robot will have much less drag than a boxy robot at higher speeds.*  This is the `Kv` term.
1. Induce a given acceleration.  This is the `Ka` term.

The [feedforward](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html#feedforward-control-in-wpilib) that we implement for the drivetrain is *velocity based*.  This means that the *input* to the feedback control is the desired speed (in meters/second) and the *output* is the corresponding voltage that needs to be sent to the motor.

#### Summary of [changes](https://github.com/BHSRobotix/RomiTutorial2023/commit/4aadeefa86de3d15f5b56336051dc1072986133c?diff=split):
1. `Arcadedrive.java`
    1. Update command to use new `arcadeDriveFeedforward()`
1. `Drivetrain.java`
    1. Create the left/right motor feedforward controllers using the left/right `Ks`, `Kv`, and `Ka` SysId constants.
    1. Implement `arcadeDriveFeedforward()` that uses the left/right feedforward to calculate voltages
        * *Feedforward expects inputs in meters/sec, so we multiply the `wheelsSpeeds` (which are **normalized** voltage levels from -1.0..1.0) by the `maxSpeedMetersPerSecond`*

### PID control
[PID control](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html) is *reactive* in that it tries to correct for errors between the desired vs actual velocity. Similar to feedforward, the drivetrain PID control is *velocity based*.

It is a feedback loop that makes adjustments *continually*. As inputs, it takes both the desired and current velocity and then it computes what it thinks is the the correct voltage to achieve the desired velocity. Internally, the PID control has a notion of the previous error terms so it knows how fast/slow it is acheiving the desired velocity and adjusts the voltages accordingly.

A WPI professor has posted a great [video explanation of PID control](https://youtu.be/UOuRx9Ujsog) and is definitely worth the watch.

#### Summary of [changes](https://github.com/BHSRobotix/RomiTutorial2023/commit/9ef225381a6b3d4e0d28a5a287c11baba2e3473a?diff=split):
1. `Arcadedrive.java`
    1. Update command to use new `arcadeDriveClosedLoop()`
1. `Drivetrain.java`
    1. Create the left/right motor PID controllers using the left/right `Kp`, `Ki`, and `Ka` SysId constants
    1. Implement `arcadeDriveClosedLoop()` that uses **both** the left/right feedforward and PID controllers to calculate voltages.
    1. Add accessors to get the left/right velocity in meters/second.
        *The PID controller needs to know the robot's current velocity to calculate the error from the desired velocity*
1. `DriveIO.java`
    1. Add left/right velocity values (in radians per second)
1.  `DriveIORomi.java`
    1. Populate the left/right velocity values (in radians per second) by reading the left/right HW encoder rate.
