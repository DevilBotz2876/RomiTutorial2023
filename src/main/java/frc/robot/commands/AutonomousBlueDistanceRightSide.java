// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class AutonomousBlueDistanceRightSide extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on time. This will drive out for a period of time, turn
   * around for time (equivalent to time to turn around) and drive forward again. This should mimic
   * driving out, turning around and driving back.
   *
   * @param drivetrain The drive subsystem on which this command will run
   */
  public AutonomousBlueDistanceRightSide(Drivetrain drivetrain) {
    addCommands(
      new DriveDistance(0.6, 8.5, drivetrain),
      new TurnDegrees(-0.4, 94, drivetrain),
      new DriveDistance(0.5, 14.5, drivetrain)
      );
  }
}
