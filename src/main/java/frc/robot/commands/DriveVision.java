package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * This class implements a command that uses PhotonVision AprilTag tracking to automatically turn
 * the robot to face the nearest/best AprilTag. If no AprilTags are detected, the robot stops
 * moving. This command never exits, so must be explicitly stopped.
 *
 * <p>Note: This class assumes there is a camera named "photonvision" and is calibrated/configured
 * to detect AprilTags
 */
public class DriveVision extends CommandBase {
  private Drivetrain drive;
  private PhotonCamera camera = new PhotonCamera("photonvision");
  private PIDController rotationPid = new PIDController(.1, 0, 0);

  public DriveVision(Drivetrain drive) {
    this.drive = drive;
    addRequirements(drive);

    rotationPid.enableContinuousInput(0, 360);
  }

  @Override
  public void initialize() {
    super.initialize();
  }

  @Override
  public void execute() {
    // Query the latest result from PhotonVision
    var visionResult = camera.getLatestResult();

    // Check if the latest result has any targets.
    boolean hasVisionTargets = visionResult.hasTargets();
    if (hasVisionTargets) {
      // Get the current best target.
      PhotonTrackedTarget visionTarget = visionResult.getBestTarget();

      // Get information from target.
      double targetYaw = visionTarget.getYaw();

      drive.setChassisSpeeds(new ChassisSpeeds(0, 0, rotationPid.calculate(targetYaw, 0)));
    } else {
      drive.stop();
    }
  }
}
