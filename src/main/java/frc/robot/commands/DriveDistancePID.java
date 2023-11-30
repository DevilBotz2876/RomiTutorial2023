// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveDistancePID extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_distance;
  private final double m_speed;
  private final PIDController anglePID;

  /**
   * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param inches The number of inches the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveDistancePID(double speed, double inches, Drivetrain drive) {
    m_distance = inches;
    m_speed = speed;
    m_drive = drive;
    anglePID = new PIDController(0.0001, 0, 0);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetGyro();
    anglePID.setSetpoint(0);
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(-anglePID.calculate(m_drive.getGyroAngleZ()));
    m_drive.arcadeDrive(m_speed, -anglePID.calculate(m_drive.getGyroAngleZ()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to desired distance
    return Math.abs(m_drive.getAverageDistanceInch()) >= m_distance;
  }
}
