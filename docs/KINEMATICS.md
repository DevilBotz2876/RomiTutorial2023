## Kinematics

Kinematics is a way to convert a drivetrain *agnostic* velocity/rotation (`ChassisSpeeds`) to values meaningful for the actual drivetrain (`WheelSpeeds`).  Similar to odometry, there are drivetrain specific classes for converting chassis speeds to wheel speeds.  Here, we use
[Differential Drive Kinematics](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-kinematics.html).  [Swerve Drive Kinematics](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html) and [Mecanum Drive Kinematics](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/mecanum-drive-kinematics.html) are also available.

### Summary of [changes](https://github.com/BHSRobotix/RomiTutorial2023/commit/3e229fce64c085362252d2a048583c52bb2c4cad?diff=split):
1. `ArcadeDrive.java`
    1. Update command to use new `arcadeDriveKinematics()`
1. `Drivetrain.java`
    1. Create a `DifferentialDriveKinematics` object that uses the robot's `trackWidthMeters` SysId constant
    1. Add a new *private* `setDriveSpeed()` helper function that accepts wheel speeds and implements [combined feedforward and PID control](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/combining-feedforward-feedback.html).
    1. Implement a new `arcadeDriveKinematics()` that uses the new `setDriveSpeed()` helper function
    1. Implement a new `setChassisSpeeds()` function that converts from a universal `ChassisSpeeds` object to wheel speeds
