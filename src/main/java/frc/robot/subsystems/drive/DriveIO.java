package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

/**
 * The DriveIO interface standardizes on radian units to represent position, velocity, and gyro
 * values. Gs are used for acceleration.
 */
public interface DriveIO {
  @AutoLog
  public static class DriveIOInputs {
    public long leftEncoderCount = 0;
    public long rightEncoderCount = 0;

    public double leftPositionInRads = 0.0;
    public double rightPositionInRads = 0.0;

    public double xAccelerationInGs = 0.0;
    public double yAccelerationInGs = 0.0;
    public double zAccelerationInGs = 0.0;

    public double xRotationInRads = 0.0;
    public double yRotationInRads = 0.0;
    public double zRotationInRads = 0.0;

    public double leftAppliedVoltage = 0.0;
    public double rightAppliedVoltage = 0.0;

    public double leftVelocityRadPerSec = 0.0;
    public double rightVelocityRadPerSec = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(DriveIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setDriveVoltage(double leftVolts, double rightVolts) {}
}
