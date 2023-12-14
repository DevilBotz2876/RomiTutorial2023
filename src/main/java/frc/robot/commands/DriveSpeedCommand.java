package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class DriveSpeedCommand extends InstantCommand {
    private final Drivetrain m_romiDrivetrain;
    private double speed;
    private boolean toggleState = false; //preserved forever/ does not reset
  
    public DriveSpeedCommand(Drivetrain drivetrain) {
      m_romiDrivetrain = drivetrain;
      this.speed = speed;
      addRequirements(drivetrain);
    }
  
    @Override
    public void initialize() {
      toggleState = !toggleState;
    
      if (toggleState) {
        // Switch to TankDrive
        speed = 0.75;
      } else {
        // Switch to ArcadeDrive
        speed = 0.5;
      }
    }    
  }
  
