package frc.robot.commands;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class DriveSwitchCommand extends InstantCommand {
    private final Drivetrain m_romiDrivetrain;
    private final CommandXboxController m_controller;
    private boolean toggleState = false; //preserved forever/ does not reset
  
    public DriveSwitchCommand(Drivetrain drivetrain, CommandXboxController controller) {
      m_romiDrivetrain = drivetrain;
      m_controller = controller;
      addRequirements(drivetrain);
    }
  
    @Override
    public void initialize() {
      toggleState = !toggleState;
  
      if (toggleState) {
        // Switch to arcade
        m_romiDrivetrain.setDefaultCommand(
          new ArcadeDrive(m_romiDrivetrain, () -> -m_controller.getLeftY(), () -> -m_controller.getRightY())
        );
      } else {
        // Switch to the second drive mode
        m_romiDrivetrain.setDefaultCommand(
          new ArcadeDrive(m_romiDrivetrain, () -> -m_controller.getLeftY(), () -> m_controller.getLeftX())
        );
      }
    }
  }
  
