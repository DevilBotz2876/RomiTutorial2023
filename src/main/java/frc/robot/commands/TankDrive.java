// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import java.util.function.Supplier;

public class TankDrive extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final Supplier<Double> m_rightZaxisRotateSupplier;
  private final Supplier<Double> m_leftZaxisSpeedSupplier;

  /**
   * Creates a new Tank Drive. This command will drive your robot according to the speed supplier
   * lambdas. This command does not terminate.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   * @param xaxisSpeedSupplier Lambda supplier of forward/backward speed
   * @param leftZaxisSpeedSupplier Lambda supplier of rotational speed
   */
  public TankDrive(
      Drivetrain drivetrain,
      Supplier<Double> leftZaxisSpeedSupplier,
      Supplier<Double> rightZaxisRotateSupplier) {
    m_drivetrain = drivetrain;
    m_leftZaxisSpeedSupplier = leftZaxisSpeedSupplier;
    m_rightZaxisRotateSupplier = rightZaxisRotateSupplier;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.tankDriveKinematics(m_leftZaxisSpeedSupplier.get(), m_rightZaxisRotateSupplier.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
