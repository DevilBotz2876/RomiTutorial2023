package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;


public class AutoRed2 extends SequentialCommandGroup {
    public AutoRed2(Drivetrain drivetrain) {
      addCommands(
          new WaitCommand(6),
          new DriveDistance(0.5, 7.75, drivetrain),
          new TurnDegrees(-0.4, 80, drivetrain),
          new DriveDistance(-1, 4, drivetrain),
          new DriveDistance(-0.5, 4, drivetrain),
          new DriveDistance(-0.5, 2, drivetrain));
    }
  }