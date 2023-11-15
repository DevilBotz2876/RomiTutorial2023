// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SysId;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends SubsystemBase {
  private static final double kWheelDiameterInch = 2.75591; // 70 mm

  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  private double leftPositionOffset;
  private double rightPositionOffset;
  private double xAccelerationOffset;
  private double yAccelerationOffset;
  private double zAccelerationOffset;

  // START: Setup Feedforward
  private SimpleMotorFeedforward leftFeedforward;
  private SimpleMotorFeedforward rightFeedforward;
  // END: Setup Feedforward

  // START: Setup PID Control feedback loop
  private PIDController leftVelocityPid;
  private PIDController rightVelocityPid;
  // END: Setup PID Control feedback loop

  // START: Setup Odometry
  private final DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(new Rotation2d(), 0.0, 0.0);
  private Field2d field = new Field2d();
  // END: Setup Odometry

  // START: Setup Kinematics
  DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(SysId.Drive.trackWidthMeters);
  // END: Setup Kinematics

  // START: Setup pathplanner
  private SimpleMotorFeedforward combinedFeedforward;
  // END: Setup pathplanner

  /**
   * Creates a new Drivetrain.
   *
   * @param io an instance that implements the DriveIO interface
   */
  public Drivetrain(DriveIO io) {
    this.io = io;

    // START: Setup Feedforward
    leftFeedforward =
        new SimpleMotorFeedforward(
            SysId.Drive.leftKsVolts,
            SysId.Drive.leftKvVoltSecondsPerMeter,
            SysId.Drive.leftKaVoltSecondsSquaredPerMeter);
    rightFeedforward =
        new SimpleMotorFeedforward(
            SysId.Drive.rightKsVolts,
            SysId.Drive.rightKvVoltSecondsPerMeter,
            SysId.Drive.rightKaVoltSecondsSquaredPerMeter);
    // END: Setup Feedforward

    // START: Setup PID Control feedback loop
    leftVelocityPid =
        new PIDController(
            SysId.Drive.leftKpVelocity, SysId.Drive.leftKiVelocity, SysId.Drive.leftKdVelocity);
    rightVelocityPid =
        new PIDController(
            SysId.Drive.rightKpVelocity, SysId.Drive.rightKiVelocity, SysId.Drive.rightKdVelocity);
    // END: Setup PID Control feedback loop

    // START: Setup Odometry
    SmartDashboard.putData("Field", field);
    // END: Setup Odometry

    // START: Setup pathplanner
    combinedFeedforward =
        new SimpleMotorFeedforward(
            SysId.Drive.combinedksVolts,
            SysId.Drive.combinedkvVoltSecondsPerMeter,
            SysId.Drive.combinedkaVoltSecondsSquaredPerMeter);
    // END: Setup pathplanner
  }

  /**
   * The Arcade Drive mode is used to control the drivetrain using speed/throttle and rotation rate.
   * This is typically used either with two axes from a single joystick, or split across joysticks
   * (often on a single gamepad) with the throttle coming from one stick and the rotation from
   * another.
   *
   * @param xaxisSpeed the desired speed along the x-axis normalized to [-1.0 to 1.0]. Positive
   *     values indicate moving forward.
   * @param zaxisRotate the desired rotation along the z-axis normalized to [-1.0 to 1.0]. Positive
   *     values indicate moving counter-clockwise.
   */
  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    var wheelSpeeds = DifferentialDrive.arcadeDriveIK(xaxisSpeed, zaxisRotate, true);
    io.setDriveVoltage(
        wheelSpeeds.left * RobotController.getBatteryVoltage(),
        wheelSpeeds.right * RobotController.getBatteryVoltage());
  }

  public void tankDrive(double xaxisSpeed, double zaxisRotate) {
    var wheelSpeeds = DifferentialDrive.arcadeDriveIK(xaxisSpeed, zaxisRotate, true);
    io.setDriveVoltage(
        wheelSpeeds.left * RobotController.getBatteryVoltage(),
        wheelSpeeds.right * RobotController.getBatteryVoltage());
  }

  // START: Setup Feedforward
  /**
   * This method is similar to {@link #arcadeDrive(double, double)}, with the following differences:
   *
   * <ul>
   *   <li>uses feedforward control to compute voltages based on the desired left/right velocity
   * </ul>
   */
  public void arcadeDriveFeedforward(double xaxisSpeed, double zaxisRotate) {
    var wheelSpeeds = DifferentialDrive.arcadeDriveIK(xaxisSpeed, zaxisRotate, true);
    io.setDriveVoltage(
        leftFeedforward.calculate(wheelSpeeds.left * SysId.Drive.maxSpeedMetersPerSecond),
        rightFeedforward.calculate(wheelSpeeds.right * SysId.Drive.maxSpeedMetersPerSecond));
  }
  // END: Setup Feedforward

  // START: Setup PID Control feedback loop
  /**
   * This method is similar to {@link #arcadeDriveFeedforward(double, double)}, with the following
   * differences:
   *
   * <ul>
   *   <li>adds PID loop feedback control to voltages computed by feedforward
   * </ul>
   */
  public void arcadeDriveClosedLoop(double xaxisSpeed, double zaxisRotate) {
    var wheelSpeeds = DifferentialDrive.arcadeDriveIK(xaxisSpeed, zaxisRotate, true);
    io.setDriveVoltage(
        leftFeedforward.calculate(wheelSpeeds.left * SysId.Drive.maxSpeedMetersPerSecond)
            + leftVelocityPid.calculate(
                getLeftVelocityMeters(), wheelSpeeds.left * SysId.Drive.maxSpeedMetersPerSecond),
        rightFeedforward.calculate(wheelSpeeds.right * SysId.Drive.maxSpeedMetersPerSecond)
            + rightVelocityPid.calculate(
                getRightVelocityMeters(), wheelSpeeds.right * SysId.Drive.maxSpeedMetersPerSecond));
  }
  // END: Setup PID Control feedback loop

  // START: Setup kinematics
  private void setDriveSpeed(DifferentialDriveWheelSpeeds speeds) {
    io.setDriveVoltage(
        leftFeedforward.calculate(speeds.leftMetersPerSecond)
            + leftVelocityPid.calculate(getLeftVelocityMeters(), speeds.leftMetersPerSecond),
        rightFeedforward.calculate(speeds.rightMetersPerSecond)
            + rightVelocityPid.calculate(getRightVelocityMeters(), speeds.rightMetersPerSecond));
  }

  /**
   * This method is similar to {@link #arcadeDriveClosedLoop(double, double)}, with the following
   * differences:
   *
   * <ul>
   *   <li>refactored to use DifferentialDriveWheelSpeeds kinematics
   * </ul>
   */
  public void arcadeDriveKinematics(double xaxisSpeed, double zaxisRotate) {
    var wheelSpeeds = DifferentialDrive.arcadeDriveIK(xaxisSpeed, zaxisRotate, true);
    setDriveSpeed(
        new DifferentialDriveWheelSpeeds(
            wheelSpeeds.left * SysId.Drive.maxSpeedMetersPerSecond,
            wheelSpeeds.right * SysId.Drive.maxSpeedMetersPerSecond));
  }

  public void tankDriveKinematics(double xaxisSpeed, double zaxisRotate) {
    var wheelSpeeds = DifferentialDrive.tankDriveIK(xaxisSpeed, zaxisRotate, true);
    setDriveSpeed(
        new DifferentialDriveWheelSpeeds(
            wheelSpeeds.left * SysId.Drive.maxSpeedMetersPerSecond,
            wheelSpeeds.right * SysId.Drive.maxSpeedMetersPerSecond));
  }
  /**
   * This method allows the use drivetrain agnostic ChassisSpeeds to move the drivetrain. This
   * allows upper level code to control the drivetrain without any assumption on the drivetrain type
   * (e.g. Differential, Swerve, Mecanum, etc.). Kinematics are used to convert chassis speeds to
   * values useful for the actual drivetrain implementation.
   *
   * @param chassisSpeeds the desired chassis speed
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    var wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

    setDriveSpeed(wheelSpeeds);
  }
  // END: Setup kinematics

  /** Reset the encoders */
  public void resetEncoders() {
    leftPositionOffset = inputs.leftPositionInRads;
    rightPositionOffset = inputs.rightPositionInRads;
  }

  /**
   * The left wheel distance in inches
   *
   * <p>Note: This is for backwards compatibiity. New code should use {@link
   * #getLeftPositionMeters()} instead.
   *
   * @return the left wheel distance (in inches)
   */
  public double getLeftDistanceInch() {
    return (inputs.leftPositionInRads - leftPositionOffset) * kWheelDiameterInch / 2;
  }

  /**
   * The right wheel distance in inches
   *
   * <p>Note: This is for backwards compatibiity. New code should use {@link
   * #getRightPositionMeters()} instead.
   *
   * @return the right wheel distance (in inches)
   */
  public double getRightDistanceInch() {
    return (inputs.rightPositionInRads - rightPositionOffset) * kWheelDiameterInch / 2;
  }

  /**
   * The average wheel distance in inches
   *
   * @return the average wheel distance (in inches)
   */
  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return inputs.xAccelerationInGs;
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return inputs.yAccelerationInGs;
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return inputs.zAccelerationInGs;
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return Units.radiansToDegrees(inputs.xRotationInRads - xAccelerationOffset);
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return Units.radiansToDegrees(inputs.yRotationInRads - yAccelerationOffset);
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return Units.radiansToDegrees(inputs.zRotationInRads - zAccelerationOffset);
  }

  /** Reset the gyro. */
  public void resetGyro() {
    xAccelerationOffset = inputs.xAccelerationInGs;
    yAccelerationOffset = inputs.yAccelerationInGs;
    zAccelerationOffset = inputs.zAccelerationInGs;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // START: Setup AdvantageKit IO
    Logger logger = Logger.getInstance();
    io.updateInputs(inputs);
    logger.processInputs("Drive", inputs);

    logger.recordOutput("Drive/Left/DistanceInch", getLeftDistanceInch());
    logger.recordOutput("Drive/Right/DistanceInch", getRightDistanceInch());
    // END: Setup AdvantageKit IO

    // START: Setup Odometry
    // Update odometry and log the new pose
    Pose2d pose = odometry.update(getRotation(), getLeftPositionMeters(), getRightPositionMeters());
    logger.recordOutput("Odometry", pose);
    field.setRobotPose(pose);
    // END: Setup Odometry
  }

  // START: Setup PID Control feedback loop
  /**
   * The current velocity of the left wheels in meters/second.
   *
   * @return the current velocity of the left wheels (in meters/second)
   */
  public double getLeftVelocityMeters() {
    return inputs.leftVelocityRadPerSec * SysId.Drive.wheelRadiusMeters;
  }

  /**
   * The current velocity of the right wheels in meters/second.
   *
   * @return the current velocity of the right wheels (in meters/second)
   */
  public double getRightVelocityMeters() {
    return inputs.rightVelocityRadPerSec * SysId.Drive.wheelRadiusMeters;
  }
  // END: Setup PID Control feedback loop

  // START: Setup Odometry
  /**
   * The current position of the left wheels in meters.
   *
   * @return the current position of the left wheels (in meters)
   */
  public double getLeftPositionMeters() {
    return inputs.leftPositionInRads * SysId.Drive.wheelRadiusMeters;
  }

  /**
   * The current position of the right wheels in meters.
   *
   * @return the current position of the right wheels (in meters)
   */
  public double getRightPositionMeters() {
    return inputs.rightPositionInRads * SysId.Drive.wheelRadiusMeters;
  }

  /**
   * The current rotation in degrees.
   *
   * @return the current rotation
   */
  public Rotation2d getRotation() {
    return new Rotation2d(-inputs.zRotationInRads);
  }
  // END: Setup Odometry

  // START: Setup pathplanner
  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(getRotation(), getLeftPositionMeters(), getRightPositionMeters(), pose);
  }

  /**
   * This method generates a command that will cause the robot to follow the specificed trajectory
   *
   * @param traj the desired trajectory/path to follow
   * @param isFirstPath set to true if the robot is at the starting point of the trajectory. When
   *     true, the pose is reset to te starting pose of the trajectory.
   * @param useAllianceColor set to true to translate the trajectory to account for any
   *     alliance-specific non-symmetrical field differences. (E.g. 2023 ChargedUp)
   * @return the command that, when executed, will control the robot to follow the desired
   *     trajectory
   */
  public Command followTrajectoryCommand(
      PathPlannerTrajectory traj, boolean isFirstPath, boolean useAllianceColor) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              // Reset odometry for the first path you run during auto
              leftVelocityPid.reset();
              rightVelocityPid.reset();

              if (isFirstPath) {
                this.resetOdometry(traj.getInitialPose());
              }
            }),
        new PPRamseteCommand(
            traj,
            () -> odometry.getPoseMeters(), // Pose supplier
            new RamseteController(),
            combinedFeedforward,
            this.kinematics, // DifferentialDriveKinematics
            () ->
                new DifferentialDriveWheelSpeeds(
                    getLeftVelocityMeters(),
                    getRightVelocityMeters()), // DifferentialDriveWheelSpeeds supplier
            leftVelocityPid, // Left controller. Tune these values for your robot. Leaving them 0
            // will only use feedforwards.
            rightVelocityPid, // Right controller (usually the same values as left controller)
            (leftVolts, rightVolts) ->
                io.setDriveVoltage(leftVolts, rightVolts), // Voltage biconsumer
            useAllianceColor, // Should the path be automatically mirrored depending on alliance
            // color.
            // Optional, defaults to true
            this // Requires this drive subsystem
            ));
  }
  // END: Setup pathplanner

  // START: Setup PhotonVision
  /** Stops all movement of the drivetrain */
  public void stop() {
    io.setDriveVoltage(0, 0);
  }
  // END: Setup PhotonVision
}
