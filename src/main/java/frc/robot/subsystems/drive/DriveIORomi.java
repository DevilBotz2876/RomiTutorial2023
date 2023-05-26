// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.sensors.RomiGyro;

/**
 * The DriveIORomi class contains the Romi HW specific drivetrain initialization of the motors,
 * encoders, gyros, and accelerometer
 */
public class DriveIORomi implements DriveIO {
  private static final double kCountsPerRevolution = 1440.0;

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  /** Creates a new Drivetrain. */
  public DriveIORomi() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use radians as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((2 * Math.PI) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((2 * Math.PI) / kCountsPerRevolution);
    resetEncoders();
  }

  private void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /** {@inheritDoc} */
  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.leftEncoderCount = m_leftEncoder.get();
    inputs.rightEncoderCount = m_rightEncoder.get();

    inputs.leftPositionInRads = m_leftEncoder.getDistance();
    inputs.rightPositionInRads = m_rightEncoder.getDistance();

    inputs.xAccelerationInGs = m_accelerometer.getX();
    inputs.yAccelerationInGs = m_accelerometer.getY();
    inputs.zAccelerationInGs = m_accelerometer.getZ();

    inputs.xRotationInRads = Units.degreesToRadians(m_gyro.getAngleX());
    inputs.yRotationInRads = Units.degreesToRadians(m_gyro.getAngleY());
    inputs.zRotationInRads = Units.degreesToRadians(m_gyro.getAngleZ());

    inputs.leftAppliedVoltage = m_leftMotor.get() * RobotController.getBatteryVoltage();
    inputs.leftAppliedVoltage = m_rightMotor.get() * RobotController.getBatteryVoltage();

    // START: Setup PID Control feedback loop
    inputs.leftVelocityRadPerSec = m_leftEncoder.getRate();
    inputs.rightVelocityRadPerSec = m_rightEncoder.getRate();
    // END: Setup PID Control feedback loop
  }

  /** {@inheritDoc} */
  @Override
  public void setDriveVoltage(double leftVolts, double rightVolts) {
    m_leftMotor.setVoltage(leftVolts);
    m_rightMotor.setVoltage(rightVolts);
  }
}
