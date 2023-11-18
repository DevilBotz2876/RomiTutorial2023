package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;


public class AutoRed2 extends SequentialCommandGroup {
    public AutoRed2(Drivetrain drivetrain) {
      addCommands(
          new DriveDistance(-0.5, 1, drivetrain),
          new TurnDegrees(0.5, 90, drivetrain),
          new DriveDistance(-0.5, 1, drivetrain),
          new TurnDegrees(0.5, 90, drivetrain),
          new DriveDistance(-0.5, 1, drivetrain));
    }
  }