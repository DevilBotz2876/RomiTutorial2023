// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SysId;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousDistanceCenter;
import frc.robot.commands.AutonomousDistanceRightSide;

 import frc.robot.commands.ArmCommand;

 import frc.robot.commands.DriveVision;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;
import frc.robot.subsystems.drive.DriveIORomi;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain(new DriveIORomi());
  private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);
  // START: Setup arm
  // Create an arm sub-system
  private final Arm m_arm = new Arm();
  // END: Setup arm

  // Assumes a XBox controller plugged into channnel 0
  private final XboxController m_controller = new XboxController(0);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the hardware "overlay"
  // that is specified when launching the wpilib-ws server on the Romi raspberry pi.
  // By default, the following are available (listed in order from inside of the board to outside):
  // - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
  // - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
  // - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
  // - PWM 2 (mapped to Arduino Pin 21)
  // - PWM 3 (mapped to Arduino Pin 22)
  //
  // Your subsystem configuration should take the overlays into account

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command is arcade drive. This will run unless another command
    // is scheduled over it.
    m_drivetrain.setDefaultCommand(getArcadeDriveCommand());

    // Example of how to use the onboard IO
    Trigger onboardButtonA = new Trigger(m_onboardIO::getButtonAPressed);
    onboardButtonA
        .onTrue(new PrintCommand("Button A Pressed"))
        .onFalse(new PrintCommand("Button A Released"));

    // Setup SmartDashboard options
    m_chooser.setDefaultOption("Auto Routine Distance Center", new AutonomousDistanceCenter(m_drivetrain));
    m_chooser.addOption("Auto Routine Right Side", new AutonomousDistanceRightSide(m_drivetrain));

    // START: Setup pathplanner
    PathPlannerTrajectory traj =
        PathPlanner.loadPath(
            "Test Path",
            new PathConstraints(
                SysId.Drive.maxSpeedMetersPerSecond, SysId.Drive.maxSpeedMetersPerSecond / 2));
    m_chooser.addOption(
        "Auto Routine Test Path", m_drivetrain.followTrajectoryCommand(traj, true, false));
    // END: Setup pathplanner

    SmartDashboard.putData(m_chooser);

    // START: Setup photonvision
    Trigger xButton = new JoystickButton(m_controller, XboxController.Button.kX.value);
    xButton.whileTrue(new DriveVision(m_drivetrain));
    // END: Setup photonvision

    Trigger leftBumper = new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);
    leftBumper.onTrue(new InstantCommand(() -> { m_arm.closeGripper(); }, m_arm));
    Trigger rightBumper = new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
    rightBumper.onTrue(new InstantCommand(() -> { m_arm.openGripper(); }, m_arm));


    // START: Setup arm
    // Use the controller's right stick's forward/back (Y-axis) to control the arm base speed
    // In this case, we want "forward" = "arm up" = positive value, but forward is reported as a
    // negative value from
    // the controller's stick, so we negate the returned value.
    m_arm.setDefaultCommand(new ArmCommand(m_arm, () -> -m_controller.getLeftY()));
    // END: Setup arm
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(
        m_drivetrain, () -> -m_controller.getRightY(), () -> -m_controller.getRightX());
  }
}
