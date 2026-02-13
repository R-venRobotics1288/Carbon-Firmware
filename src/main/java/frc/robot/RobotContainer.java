// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.GoToPose;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();

  XboxController m_driverController = new XboxController(0);
  Trigger startButton = new JoystickButton(m_driverController, XboxController.Button.kStart.value);
  Trigger aButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);

  public Shuffle m_shuffle = new Shuffle();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_robotDrive.setDefaultCommand(new RunCommand(
        () -> m_robotDrive.drive(
            -MathUtil.applyDeadband(ShuffleValues.translationfiltery.calculate(m_driverController.getLeftY()),
                ShuffleValues.kDriveDeadband),
            -MathUtil.applyDeadband(ShuffleValues.translationfilterx.calculate(m_driverController.getLeftX()),
                ShuffleValues.kDriveDeadband),
            -MathUtil.applyDeadband(ShuffleValues.rotationfilter.calculate(m_driverController.getRightX()),
                ShuffleValues.kDriveDeadband),
            ShuffleValues.kfieldRelative),
        m_robotDrive));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    startButton.onTrue(Commands.runOnce(() -> m_robotDrive.resetGyro()));

    // When the 'A' button is pressed, run the GoToRelativePose command.
    // The command will be cancelled if the driver moves the joysticks.
    aButton.onTrue(
        Commands.runOnce(() -> {
          // Get the robot's current pose
          Pose2d currentPose = m_robotDrive.getPose();
          // Define the desired field-relative translation
          Translation2d fieldRelativeTranslation = new Translation2d(1.0, 0.5);
          // Create the target pose by adding the translation to the current pose's translation
          Pose2d targetPose = new Pose2d(currentPose.getTranslation().plus(fieldRelativeTranslation), currentPose.getRotation());
          // Schedule the command to go to the absolute target pose
          new GoToPose(m_robotDrive, m_driverController, targetPose).schedule();
        }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*
   * public Command getAutonomousCommand() {
   * // An example command will be run in autonomous
   * return Autos.exampleAuto(m_exampleSubsystem);
   * }
   */

  public void refresh_shuffleboard() {
    m_shuffle.refreshValue(m_robotDrive.m_frontLeft.getState().angle.getDegrees(),
        m_robotDrive.m_frontLeft.getPosition().angle.getDegrees());
  }

  public void printLimeLight() {
    System.out.println(LimelightHelpers.getTX("limelight"));
  }
}
