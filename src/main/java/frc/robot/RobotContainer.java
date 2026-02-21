package frc.robot;

import static frc.robot.Constants.AutonomousConstants.*;
import static frc.robot.Constants.HIDConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.VariableShootCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class RobotContainer {
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  public RobotContainer() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      throw new IllegalStateException("Failed to read PathPlanner config!");
    }

    AutoBuilder.configure(
        driveSubsystem::getEstimatedRobotPose,
        driveSubsystem::resetEstimatedPose,
        driveSubsystem::getChassisSpeeds,
        (speeds, feedforwards) -> driveSubsystem.drive(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(PP_DRIVE_PID_P, PP_DRIVE_PID_I, PP_DRIVE_PID_D),
            new PIDConstants(PP_TURN_PID_P, PP_TURN_PID_I, PP_TURN_PID_D)),
        config,
        () -> {
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        driveSubsystem);

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

  private final CommandXboxController driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
  private final SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(TRANSLATION_SLEW_LIMIT);
  private final SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(TRANSLATION_SLEW_LIMIT);
  private final SlewRateLimiter rotationSlewRateLimiter = new SlewRateLimiter(ROTATION_SLEW_LIMIT);

  private void configureBindings() {
    // Bind driving to the default (continuously ran) command of the driveSubsystem.
    driveSubsystem.setDefaultCommand(new RunCommand(() -> driveSubsystem.drive(
        -MathUtil.applyDeadband(
            ySlewRateLimiter.calculate(driverController.getLeftY()),
            DRIVER_CONTROLLER_DEADBAND),
        -MathUtil.applyDeadband(
            xSlewRateLimiter.calculate(driverController.getLeftX()),
            DRIVER_CONTROLLER_DEADBAND),
        -MathUtil.applyDeadband(
            rotationSlewRateLimiter.calculate(driverController.getRightX()),
            DRIVER_CONTROLLER_DEADBAND),
        true), driveSubsystem));

    driverController.start().onTrue(Commands.runOnce(driveSubsystem::zeroGyroscope));
    driverController.x().onTrue(driveSubsystem.setXCommand);
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("test");
  }
}
