package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoSubsystem extends SubsystemBase {
  private final Servo m_servo;

  /**
   * Creates a servo subsystem
   *
   * @param channel PWM channel ID for the servo
   */
  public ServoSubsystem(int channel) {
    m_servo = new Servo(channel);
  }

  /**
   * Sets the servo to the requested position
   *
   * @param angle desired angle in percentage of total range. [0..1.0]. 0 is full counterclockwise.
   *     1.0 is full clockwise.
   */
  public void set(double position) {
    m_servo.set(position);
  }
}
