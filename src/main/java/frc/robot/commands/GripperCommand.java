package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;
import java.util.function.DoubleSupplier;

public class GripperCommand extends CommandBase {
  // the gripper subsystem used by this comment
  private final Gripper m_gripper;

  // the supplier of the desired speed
  private final DoubleSupplier m_baseSpeed;

  // default start position for the gripper when the command is first scheduled
  private final double m_defaultPosition = 0.5;

  private final double m_deadband =
      0.05; // to prevent stick drift, this value sets the min absolute value the speed needs to be

  // to prevent stick drift, this value sets the min absolute value the speed needs to be
  // before we assume it is not zero
  private final double m_speedScale =
      16; // to reduce stick sensitivity, this value indicates how much to scale the returned speed
  // by
  private final double m_minBaseRange = 0; // min range for the gripper's base
  private final double m_maxBaseRange = 1; // max range for the gripper's base

  public GripperCommand(Gripper gripper, DoubleSupplier baseSpeed) {
    m_gripper = gripper;
    m_baseSpeed = baseSpeed;

    // To prevent scheduling conflicts, all commands need to indicate which sub-system(s) it uses
    addRequirements(gripper);
  }

  @Override
  public void initialize() {
    // We set the gripper's base position to the default position when this command is first
    // scheduled
    m_gripper.setClawPosition(m_defaultPosition);
  }

  @Override
  public void execute() {
    // Variable that will store the new calculated gripper position
    double newClawPosition;

    // Determine the requested speed, but ignoring inputs near-zero (i.e. +/= m_deadband)
    double newBasePositionDelta = MathUtil.applyDeadband(m_baseSpeed.getAsDouble(), m_deadband);
    // Scale the speed as desired to reduce sensitivity
    newBasePositionDelta /= m_speedScale;

    // Calculate the gripper position by adding the computer speed to the gripper base's current
    // position
    newClawPosition = m_gripper.getClawPosition() + newBasePositionDelta;

    // Clamp the gripper's upper/lower position to the min/max range allowed
    newClawPosition = MathUtil.clamp(newClawPosition, m_minBaseRange, m_maxBaseRange);

    // Finally, set the gripper base positon to the new calculated gripper position.
    m_gripper.setClawPosition(newClawPosition);
  }
}
