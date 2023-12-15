package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  // This code assumes we are using the ROMI's external IO configuration and the arm base servo is
  // connected to Romi Pin EXT4 which is configured as PWM and mapped to port 3
  // https://docs.wpilib.org/en/stable/docs/romi-robot/web-ui.html#external-io-configuration
  private final Servo m_armBaseServo = new Servo(3);

  // Note: If you have a multi-axis arm, you could create another Servo object using a differemt PWM
  // channel and add the corresponding methods for controlling the position.
  private final Servo m_armElbowServo = new Servo(2);

  /**
   * Moves the arm base to the desired position. -1.0 is max down, and 1.0 is max up
   *
   * <p>Note: The concept of "up and down" depends on how the servo is mounted to the arm. E.g. if
   * the servo is flipped, the directions would be reversed. This module abstracts the hardware
   * implementation details and follows the convention of 1.0 is up.
   *
   * @param position desired target position for arm base [0.0 to 1.0]
   */
  public void setArmBasePosition(double position) {
    // Note: This code doesn't validate the requested position.  If we don't want the arm to move
    // within the entire range (e.g. due to physical constraints), we could clamp the position to
    // the min/max values before setting the servo's position
    //System.out.println(position);
    position = MathUtil.clamp(position, 0, 0.43);
    m_armBaseServo.set(position);
    
  }

  /**
   * @return the current desired target position for arm base. Note: since the basic servo doesn't
   *     have any feedback, this is just the _requested_ position. The actual servo position is not
   *     known.
   */
  public double getArmBasePosition() {
    return m_armBaseServo.get();
  }

  public void closeGripper() {
    m_armElbowServo.set(0.50);
  }
  public void openGripper() {
      m_armElbowServo.set(1);  
    }
  }

