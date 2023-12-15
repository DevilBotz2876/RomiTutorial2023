// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class AutonomousRedDistanceCenter extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousRedDistanceCenter(Drivetrain drivetrain) {
    addCommands(
        new DriveDistance(0.6, 14.4, drivetrain),
        new TurnDegrees(0.35, 87, drivetrain),
        new DriveDistance(0.5, 5.5, drivetrain)
        );
  }
}
