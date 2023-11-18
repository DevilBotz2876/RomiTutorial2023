package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;


public class AutoRed1 extends SequentialCommandGroup {
    /**
     * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
     * turn around and drive back.
     *
     * @param drivetrain The drivetrain subsystem on which this command will run
     */
    public AutoRed1(Drivetrain drivetrain) {
      addCommands(
          new DriveDistance(-0.5, 1, drivetrain),
          new TurnDegrees(0.5, 90, drivetrain),
          new DriveDistance(-0.5, 1, drivetrain));
    }
  }