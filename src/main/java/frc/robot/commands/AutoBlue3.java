package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;


public class AutoBlue3 extends SequentialCommandGroup {
    public AutoBlue3(Drivetrain drivetrain) {
      addCommands(
        new DriveDistancePID(0.5, 12, drivetrain));
      }
  }