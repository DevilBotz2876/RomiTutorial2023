package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class DriveSwitchCommand extends InstantCommand {
    private final Drivetrain m_romiDrivetrain;
    private final CommandXboxController m_controller;
    private DriveSpeedCommand driveSpeedCommand;
    private boolean toggleState = false; //preserved forever/ does not reset
  
    public DriveSwitchCommand(Drivetrain drivetrain, CommandXboxController controller, DriveSpeedCommand driveSpeedCommand) {
      m_romiDrivetrain = drivetrain;
      m_controller = controller;
      this.driveSpeedCommand = driveSpeedCommand;
      addRequirements(drivetrain);
    }
  
    @Override
    public void initialize() {
      toggleState = !toggleState;
      
      double speed = driveSpeedCommand.getSpeed();

      if (toggleState) {
        // Switch to TankDrive
        m_romiDrivetrain.setDefaultCommand(
          new TankDrive(m_romiDrivetrain, () -> -m_controller.getLeftY(), () -> -m_controller.getRightY(), speed)
        );
        System.out.println("tank");
      } else {
        // Switch to ArcadeDrive
        m_romiDrivetrain.setDefaultCommand(
          new ArcadeDrive(m_romiDrivetrain, () -> -m_controller.getLeftY(), () -> m_controller.getLeftX(), speed)
        );
        System.out.println("arcade");
      }
      
    }

  }
  
