// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
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
  public final HopperSubsystem m_hopper = new HopperSubsystem();

  XboxController m_driverController = new XboxController(0);
  XboxController m_operatorController = new XboxController(1);

  Trigger startButton = new JoystickButton(m_driverController, XboxController.Button.kStart.value);
  Trigger shootButton = new JoystickButton(m_operatorController, XboxController.Button.kX.value);
  Trigger intakeButton = new JoystickButton(m_operatorController, XboxController.Button.kY.value);
  Trigger rotateButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
  Trigger forwardButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);
  Trigger leftButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);

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
            -MathUtil.applyDeadband(DriveConstants.translationfiltery.calculate(m_driverController.getLeftY()),
                DriveConstants.kDriveDeadband),
            -MathUtil.applyDeadband(DriveConstants.translationfilterx.calculate(m_driverController.getLeftX()),
                DriveConstants.kDriveDeadband),
            -MathUtil.applyDeadband(DriveConstants.rotationfilter.calculate(m_driverController.getRightX()),
                DriveConstants.kDriveDeadband),
            DriveConstants.kfieldRelative),
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

    shootButton.whileTrue(new ShootCommand(m_hopper));
    intakeButton.whileTrue(new IntakeCommand(m_hopper));
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
  
  Pose2d startPose = null;
  public Command getAutonomousCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    //return new RunCommand(() -> {
    //  if (startPose == null) {
    //    startPose = m_robotDrive.m_poseEstimator.getEstimatedPosition();
    //    m_robotDrive.drive(0.0, -0.3, 0, false);
    //    return;
    //  }
    //  Transform2d cur = m_robotDrive.m_poseEstimator.getEstimatedPosition().minus(startPose);
    //  if (Math.abs(cur.getX()) >= 2 || Math.abs(cur.getY()) >= 2) {
    //    m_robotDrive.drive(0.0, 0.0, 0, false);
    //    return;
    //  }
    //  System.out.println("X, Y: " + cur.getX() + ", " + cur.getY());
    //  m_robotDrive.drive(0.0, -0.4, 0, false);
    //}, m_robotDrive);
    return new PathPlannerAuto("test");
  }

  public void refresh_shuffleboard() {
    m_shuffle.refreshValue(m_robotDrive.m_frontLeft.getState().angle.getRadians(),
        m_robotDrive.m_frontLeft.getPosition().angle.getRadians());
        
    m_shuffle.refreshValue(m_robotDrive.m_frontRight.getState().angle.getRadians(),
        m_robotDrive.m_frontRight.getPosition().angle.getRadians());
    m_field.setRobotPose(m_robotDrive.getPose());
  }

  public void printLimeLight() {
    System.out.println(LimelightHelpers.getTX(""));
  }
}
