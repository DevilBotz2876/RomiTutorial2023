package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class setDefaultArmCommand extends CommandBase{
    

    public setDefaultArmCommand(Arm arm,Command cm) {
        arm.setDefaultCommand(cm);
        addRequirements(arm);
        System.out.println("hi");
    }

}
