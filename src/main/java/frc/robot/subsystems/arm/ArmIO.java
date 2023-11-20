package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double shoulderPosition;
    public double elbowPosition;
  }

  /**
   * Updates the set of loggable inputs. Should be called periodically.
   *
   * @param inputs the set of loggable inputs
   */
  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setArmPosition(double shoulderPosition, double elbowPosition) {}
}
