// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class AutonomousDistance extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousDistance(Drivetrain drivetrain) {
    addCommands(
      //This is the code for the red side in front of the ramp.
        new DriveDistance(0.7, 2, drivetrain));
        //new TurnDegrees(0.5, 255, drivetrain),
        //new DriveDistance(1, 2.4, drivetrain),
        //new TurnDegrees(0.5, 0, drivetrain));
      
      
        //This is the code for the red side on left
        /*new DriveDistance(-1, 7, drivetrain),
        new TurnDegrees(0.5, 263, drivetrain),
        new DriveDistance(0.7, 12, drivetrain));*/

        //This is code for blue alliance ramp
        /*new DriveDistance(-0.7, 11, drivetrain),
        new TurnDegrees(0.5, 90, drivetrain),
        new DriveDistance(0.7, 4, drivetrain));*/

        //This is code for the blue alliance away from ramp
        /*new DriveDistance(-1, 5, drivetrain),
        new TurnDegrees(0.5, 73, drivetrain),
        new DriveDistance(1, 11, drivetrain));*/



        
    
    
  }
}
