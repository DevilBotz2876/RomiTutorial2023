package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import java.util.function.DoubleSupplier;

public class ArmCommand extends CommandBase {
  // the arm subsystem used by this comment
  private final Arm m_arm;

  // the supplier of the desired speed
  private final DoubleSupplier m_baseSpeed;

  // default start position for the arm when the command is first scheduled
  private final double m_defaultPosition = 0.5;

  private final double m_deadband =
      0.005; // to prevent stick drift, this value sets the min absolute value the speed needs to be

  // to prevent stick drift, this value sets the min absolute value the speed needs to be
  // before we assume it is not zero
  private final double m_speedScale =
      100; // to reduce stick sensitivity, this value indicates how much to scale the returned speed
  // by
  private final double m_minBaseRange = 0; // min range for the arm's base
  private final double m_maxBaseRange = 0.6; // max range for the arm's base

  public ArmCommand(Arm arm, DoubleSupplier baseSpeed) {
    m_arm = arm;
    m_baseSpeed = baseSpeed;

    // To prevent scheduling conflicts, all commands need to indicate which sub-system(s) it uses
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    // We set the arm's base position to the default position when this command is first scheduled
    m_arm.setArmBasePosition(m_defaultPosition);
  }

  @Override
  public void execute() {
    // Variable that will store the new calculated arm position
    double newArmPosition;

    // Determine the requested speed, but ignoring inputs near-zero (i.e. +/= m_deadband)
    double newBasePositionDelta = MathUtil.applyDeadband(m_baseSpeed.getAsDouble(), m_deadband);
    // Scale the speed as desired to reduce sensitivity
    newBasePositionDelta /= m_speedScale;

    // Calculate the arm position by adding the computer speed to the arm base's current position
    newArmPosition = m_arm.getArmBasePosition() + newBasePositionDelta;

    // Clamp the arm's upper/lower position to the min/max range allowed
    newArmPosition = MathUtil.clamp(newArmPosition, m_minBaseRange, m_maxBaseRange);

    // Finally, set the arm base positon to the new calculated arm position.
    m_arm.setArmBasePosition(newArmPosition);
  }
}