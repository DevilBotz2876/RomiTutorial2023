// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends SubsystemBase {
  private static final double kWheelDiameterInch = 2.75591; // 70 mm

  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  private long leftEncodeOffset;
  private long rightEncoderOffset;
  private double leftPositionOffset;
  private double rightPositionOffset;
  private double xAccelerationOffset;
  private double yAccelerationOffset;
  private double zAccelerationOffset;

  /** Creates a new Drivetrain. */
  public Drivetrain(DriveIO io) {
    this.io = io;
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    var wheelSpeeds = DifferentialDrive.arcadeDriveIK(xaxisSpeed, zaxisRotate, true);
    io.setDriveVoltage(
        wheelSpeeds.left * RobotController.getBatteryVoltage(),
        wheelSpeeds.right * RobotController.getBatteryVoltage());
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
  }
}
