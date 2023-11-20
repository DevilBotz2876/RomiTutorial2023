package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.Servo;

public class ArmIORomi implements ArmIO {
  private final Servo m_servoShoulder = new Servo(2);
  private final Servo m_servoElbow = new Servo(3);

  public ArmIORomi() {}

  public void updateInputs(ArmIOInputs inputs) {
    inputs.shoulderPosition = m_servoShoulder.getPosition();
    inputs.elbowPosition = m_servoElbow.getPosition();
  }

  public void setArmPosition(double shoulderPosition, double elbowPosition) {
    m_servoShoulder.set(shoulderPosition);
    m_servoElbow.set(elbowPosition);
  }
}
