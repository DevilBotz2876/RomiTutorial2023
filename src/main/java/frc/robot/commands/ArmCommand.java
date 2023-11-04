package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ServoSubsystem;
import java.util.function.DoubleSupplier;

public class ArmCommand extends CommandBase {
  private final ServoSubsystem m_servoSubsystem;
  private final DoubleSupplier m_speed;
  private double m_currentPosition;
  private final double m_deadband = 0.05;
  private final double m_minRange = -1.0;
  private final double m_maxRange = 1.0;
  private final double m_speedScale = 8; // How much to divide the speed by.

  /**
   * Creates a servo control command
   *
   * @param servoSubsystem ServoSubsustem instance
   * @param position The arm's speed 1.0 corresponds to max up. -1.0 corresponds to max down.
   */
  public ArmCommand(ServoSubsystem servoSubsystem, DoubleSupplier speed) {
    m_servoSubsystem = servoSubsystem;
    m_speed = speed;

    addRequirements(servoSubsystem);
  }

  @Override
  public void initialize() {
    m_currentPosition = m_maxRange;
    m_servoSubsystem.set(m_currentPosition);
  }

  @Override
  public void execute() {
    double m_currentSpeed =
        MathUtil.applyDeadband(m_speed.getAsDouble(), m_deadband) / m_speedScale;
    if (m_currentSpeed != 0) {
      m_currentPosition =
          MathUtil.clamp(m_currentPosition + m_currentSpeed, m_minRange, m_maxRange);
      // System.out.printf("speed: %f --> position: %f\n", m_currentSpeed, m_currentPosition);
      m_servoSubsystem.set(m_currentPosition);
    }
  }
}
