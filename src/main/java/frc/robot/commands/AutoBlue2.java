package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;


public class AutoBlue2 extends SequentialCommandGroup {
    public AutoBlue2(Drivetrain drivetrain) {
      addCommands(
          new DriveDistance(0.5, 7.5, drivetrain),
          new TurnDegrees(0.4, 80, drivetrain),
          new DriveDistance(-1, 4, drivetrain),
          new DriveDistance(-0.5, 4, drivetrain),
          new DriveDistance(-0.5, 2, drivetrain),
          new DriveDistance(0.5, 0.5, drivetrain));
    }
  }