package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public Arm(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // START: Setup AdvantageKit IO
    Logger logger = Logger.getInstance();
    io.updateInputs(inputs);
    logger.processInputs("Arm", inputs);
    // END: Setup AdvantageKit IO
  }

  public void setArmPosition(double shoulderDegrees, double elbowDegrees) {
    io.setArmPosition(shoulderDegrees, elbowDegrees);
  }

  public double getShoulderPosition() {
    return inputs.shoulderPosition;
  }

  public double getElbowPosition() {
    return inputs.elbowPosition;
  }
}
