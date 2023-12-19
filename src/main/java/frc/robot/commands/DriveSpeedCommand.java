package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DriveSpeedCommand extends InstantCommand {
    private double speed = 0.5;
    private boolean toggleState = false; //preserved forever/ does not reset
  
    public DriveSpeedCommand() {
    }
  
    @Override
    public void initialize() {
      toggleState = !toggleState;
    
      if (toggleState) {
        // Switch to TankDrive
        speed = 1.25;
      } else {
        // Switch to ArcadeDrive
        speed = 0.5;
      }
      
    }  


    public double getSpeed() {
      return speed;
    }
  }
  
