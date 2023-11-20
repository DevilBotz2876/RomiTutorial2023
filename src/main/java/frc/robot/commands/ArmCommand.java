package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import java.util.function.DoubleSupplier;

public class ArmCommand extends CommandBase {
  private final Arm m_arm;
  private final DoubleSupplier m_shoulderSpeed;
  private double m_currentShoulderPosition;
  private final DoubleSupplier m_elbowSpeed;
  private double m_currentElbowPosition;

  private final double m_deadband = 0.05;
  private final double m_minRange = -1;
  private final double m_maxRange = 1;
  private final double m_speedScale = 16; // How much to divide the speed by.

  /**
   * Creates a servo control command
   *
   * @param servoSubsystem ServoSubsustem instance
   * @param position The arm's speed 1.0 corresponds to max up. -1.0 corresponds to max down.
   */
  public ArmCommand(Arm arm, DoubleSupplier shoulderSpeed, DoubleSupplier elbowSpeed) {
    m_arm = arm;
    m_shoulderSpeed = shoulderSpeed;
    m_elbowSpeed = elbowSpeed;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    m_currentShoulderPosition = 0.5;
    m_currentElbowPosition = 0.5;
    m_arm.setArmPosition(m_currentShoulderPosition, m_currentElbowPosition);
  }

  @Override
  public void execute() {

    /* Limits
     *  Shoulder: 0 = max down, 1 = max up
     *  Elbow: 0 = max up, 1 = max down
     *  Elbow Max 1.0 when Shoulder 0.20
     */
    double m_currentShoulderSpeed =
        MathUtil.applyDeadband(m_shoulderSpeed.getAsDouble(), m_deadband) / m_speedScale;
    double m_currentElbowSpeed =
        MathUtil.applyDeadband(m_elbowSpeed.getAsDouble(), m_deadband) / m_speedScale;

    if ((m_currentShoulderSpeed != 0) || (m_currentElbowSpeed != 0)) {
      double newShoulderPosition =
          MathUtil.clamp(
              m_currentShoulderPosition + m_currentShoulderSpeed, m_minRange, m_maxRange);
      double deltaShoulderPosition = newShoulderPosition - m_currentShoulderPosition;
      m_currentShoulderPosition = newShoulderPosition;

      m_currentElbowPosition =
          MathUtil.clamp(
              m_currentElbowPosition + m_currentElbowSpeed + deltaShoulderPosition,
              m_minRange,
              m_maxRange);
      // System.out.printf("speed: %f --> position: %f\n", m_currentSpeed, m_currentPosition);
      m_arm.setArmPosition(m_currentShoulderPosition, m_currentElbowPosition);
    }
  }
}
