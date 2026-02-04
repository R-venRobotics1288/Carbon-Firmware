// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;

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

  public Shuffle m_shuffle = new Shuffle();

  public LimelightHelpers.PoseEstimate limelightMeasurement;

  private final Field2d m_field = new Field2d();

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
    LimelightHelpers.SetRobotOrientation("limelight",
        m_robotDrive.m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
    m_robotDrive.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
    m_robotDrive.m_poseEstimator.addVisionMeasurement(
        limelightMeasurement.pose,
        limelightMeasurement.timestampSeconds);
    SmartDashboard.putData("Field", m_field);
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
    startButton.onTrue(Commands.runOnce(() -> {
      m_robotDrive.resetGyro();
    }, m_robotDrive));
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

  public void updateOdometry() {
    m_robotDrive.periodic();
    LimelightHelpers.SetRobotOrientation("limelight",
        m_robotDrive.m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
    if (limelightMeasurement.tagCount >= 2) { // Only trust measurement if we see multiple tags
      m_robotDrive.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
      m_robotDrive.m_poseEstimator.addVisionMeasurement(
          limelightMeasurement.pose,
          limelightMeasurement.timestampSeconds);
    }
  }

  public void refresh_shuffleboard() {
    m_shuffle.refreshValue(m_robotDrive.m_frontLeft.getState().angle.getRadians(),
        m_robotDrive.m_frontLeft.getPosition().angle.getRadians());
    m_field.setRobotPose(m_robotDrive.getPose());
  }

  public void printLimeLight() {
    System.out.println(LimelightHelpers.getTX(""));
  }
}
