// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class AutonomousDistanceCenter extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousDistanceCenter(Drivetrain drivetrain) {
    addCommands(
        new DriveDistance(0.75, 14.5, drivetrain),
        new TurnDegrees(-0.35, 90, drivetrain),
        new DriveDistance(0.5, 4.5, drivetrain)
        //new TurnDegrees(0.5, 180, drivetrain)
        );
  }
}
