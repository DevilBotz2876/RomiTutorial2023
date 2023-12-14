package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class PushCommand extends CommandBase {
    private final double pos;
    private final Arm arm;
    
    public PushCommand(double pos, Arm arm) {
        this.pos = pos;
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        arm.setArmBasePosition(pos);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return arm.getArmBasePosition() == pos;
    }
  }