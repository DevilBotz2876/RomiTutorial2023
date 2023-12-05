package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;


public class AutoRed3 extends SequentialCommandGroup {
    public AutoRed3(Drivetrain drivetrain) {
      addCommands(
          //new DriveDistancePID(0.5, 14, drivetrain),
          //new TurnDegrees(0.37, 87, drivetrain),
          new DriveDistancePID(0.5, 19, drivetrain));
          /*new TurnDegrees(0.5, 90, drivetrain),
          new DriveDistance(-1, 16, drivetrain));*/
    }
  }