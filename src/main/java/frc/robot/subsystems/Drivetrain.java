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
  // START: Setup SysId
  private static final double kWheelRadiusMeter = 0.070 / 2;
  // END: Setup SysId

  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  private long leftEncodeOffset;
  private long rightEncoderOffset;
  private double leftPositionOffset;
  private double rightPositionOffset;
  private double xAccelerationOffset;
  private double yAccelerationOffset;
  private double zAccelerationOffset;

  // START: Setup SysId
  private SimpleMotorFeedforward leftFeedforward;
  private SimpleMotorFeedforward rightFeedforward;

  private PIDController leftVelocityPid;
  private PIDController rightVelocityPid;
  // END: Setup SysId

  // START: Setup Odometry
  private final DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(new Rotation2d(), 0.0, 0.0);
  private Field2d field = new Field2d();
  // END: Setup Odometry

  // START: Setup PathPlanner
  DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(SysId.Drive.trackWidthMeters);
  private SimpleMotorFeedforward combinedFeedforward;
  // END: Setup PathPlanner

  /** Creates a new Drivetrain. */
  public Drivetrain(DriveIO io) {
    this.io = io;

    // START: Setup SysId
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

    leftVelocityPid =
        new PIDController(
            SysId.Drive.leftKpVelocity, SysId.Drive.leftKiVelocity, SysId.Drive.leftKdVelocity);
    rightVelocityPid =
        new PIDController(
            SysId.Drive.rightKpVelocity, SysId.Drive.rightKiVelocity, SysId.Drive.rightKdVelocity);
    // END: Setup SysId

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

  public void arcadeDriveOpenLoop(double xaxisSpeed, double zaxisRotate) {
    var wheelSpeeds = DifferentialDrive.arcadeDriveIK(xaxisSpeed, zaxisRotate, true);
    io.setDriveVoltage(
        wheelSpeeds.left * RobotController.getBatteryVoltage(),
        wheelSpeeds.right * RobotController.getBatteryVoltage());
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    var wheelSpeeds = DifferentialDrive.arcadeDriveIK(xaxisSpeed, zaxisRotate, true);
    setSpeeds(
        new DifferentialDriveWheelSpeeds(
            wheelSpeeds.left * SysId.Drive.maxSpeedMetersPerSecond,
            wheelSpeeds.right * SysId.Drive.maxSpeedMetersPerSecond));
  }

  public void resetEncoders() {
    leftEncodeOffset = inputs.leftEncoderCount;
    rightEncoderOffset = inputs.rightEncoderCount;
    leftPositionOffset = inputs.leftPositionInRads;
    rightPositionOffset = inputs.rightPositionInRads;
  }

  public int getLeftEncoderCount() {
    return (int) (inputs.leftEncoderCount - leftEncodeOffset);
  }

  public int getRightEncoderCount() {
    return (int) (inputs.rightEncoderCount - rightEncoderOffset);
  }

  public double getLeftDistanceInch() {
    return (inputs.leftPositionInRads - leftPositionOffset) * kWheelDiameterInch / 2;
  }

  public double getRightDistanceInch() {
    return (inputs.rightPositionInRads - rightPositionOffset) * kWheelDiameterInch / 2;
  }

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

  // START: Setup SysId
  /** Returns the velocity of the left wheels in meters/second. */
  public double getLeftVelocityMeters() {
    return inputs.leftVelocityRadPerSec * kWheelRadiusMeter;
  }

  /** Returns the velocity of the right wheels in meters/second. */
  public double getRightVelocityMeters() {
    return inputs.rightVelocityRadPerSec * kWheelRadiusMeter;
  }

  private void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    // Calculates the desired voltages for the left and right sides of the drive train.
    final double leftFeedforward = this.leftFeedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = this.rightFeedforward.calculate(speeds.rightMetersPerSecond);

    // Calculates the PID output for the left and right sides of the drive train.
    final double leftOutput =
        leftVelocityPid.calculate(getLeftVelocityMeters(), speeds.leftMetersPerSecond);
    final double rightOutput =
        rightVelocityPid.calculate(getRightVelocityMeters(), speeds.rightMetersPerSecond);

    // Sets the motor controller speeds.
    io.setDriveVoltage(leftOutput + leftFeedforward, rightOutput + rightFeedforward);
  }
  // END: Setup SysId

  // START: Setup Odometry
  /** Returns the position of the left wheels in meters. */
  public double getLeftPositionMeters() {
    return inputs.leftPositionInRads * kWheelRadiusMeter;
  }

  /** Returns the position of the right wheels in meters. */
  public double getRightPositionMeters() {
    return inputs.rightPositionInRads * kWheelRadiusMeter;
  }

  /** Returns the rotation in degrees. */
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

  /** Returns the current odometry pose in meters. */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocityMeters(), getRightVelocityMeters());
  }

  public void setDriveVoltage(double leftVolts, double rightVolts) {
    io.setDriveVoltage(leftVolts, rightVolts);
  }

  // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
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
            this::getPose, // Pose supplier
            new RamseteController(),
            combinedFeedforward,
            this.kinematics, // DifferentialDriveKinematics
            this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
            leftVelocityPid, // Left controller. Tune these values for your robot. Leaving them 0
            // will only use feedforwards.
            rightVelocityPid, // Right controller (usually the same values as left controller)
            this::setDriveVoltage, // Voltage biconsumer
            useAllianceColor, // Should the path be automatically mirrored depending on alliance
            // color.
            // Optional, defaults to true
            this // Requires this drive subsystem
            ));
  }
  // END: Setup pathplanner
}
