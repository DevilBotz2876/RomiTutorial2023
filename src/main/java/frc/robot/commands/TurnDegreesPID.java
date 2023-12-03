// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TurnDegreesPID extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_degrees;
  private final double m_angleSpeed;
  private final PIDController speedPID;

  /**
   * Creates a new TurnDegrees. This command will turn your robot for a desired rotation (in
   * degrees) and rotational speed.
   *
   * @param speed The speed which the robot will drive. Negative is in reverse.
   * @param degrees Degrees to turn. Leverages encoders to compare distance.
   * @param drive The drive subsystem on which this command will run
   */
  public TurnDegreesPID(double speed, double degrees, Drivetrain drive) {
    m_degrees = degrees;
    m_angleSpeed = speed;
    m_drive = drive;
    speedPID = new PIDController(0.00001, 0, 0);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set motors to stop, read encoder values for starting point
    speedPID.setSetpoint(m_drive.getGyroAngleY()+m_degrees);
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(-speedPID.calculate(m_drive.getGyroAngleY()));
    m_drive.arcadeDrive(-speedPID.calculate(m_drive.getGyroAngleY()), m_angleSpeed);

    double currentGyroAngle = m_drive.getGyroAngleY();
    double pidOutput = -speedPID.calculate(currentGyroAngle);
    System.out.println("Gyro Angle: " + currentGyroAngle + ", PID Output: " + pidOutput);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentGyroAngle = m_drive.getGyroAngleY();
    double targetGyroAngle = m_drive.getGyroAngleY() + m_degrees;

    // Check if the current gyro angle is greater than or equal to the target gyro angle
    return currentGyroAngle >= targetGyroAngle;

  }
}
