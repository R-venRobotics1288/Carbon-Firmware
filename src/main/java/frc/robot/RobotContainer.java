package frc.robot;

import static frc.robot.Constants.AutonomousConstants.*;
import static frc.robot.Constants.HIDConstants.*;
import static frc.robot.Constants.ClimberConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import frc.robot.utilities.Dashboard;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HopperSubsystem;

import frc.robot.commands.VariableShootCommand;

public class RobotContainer {
  private final Dashboard dashboard = new Dashboard();

  private final DriveSubsystem driveSubsystem = new DriveSubsystem(dashboard);
  //private final HopperSubsystem hopperSubsystem = new HopperSubsystem();
  //private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

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
  }

  private final CommandXboxController driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);

  private final SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(TRANSLATION_SLEW_LIMIT);
  private final SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(TRANSLATION_SLEW_LIMIT);
  private final SlewRateLimiter rotationSlewRateLimiter = new SlewRateLimiter(ROTATION_SLEW_LIMIT);

  //private final Command climbCommand = Commands.sequence(
  //    climberSubsystem.climbCommand(CLIMBER_POSITION_ONE, true),
  //    climberSubsystem.climbCommand(CLIMBER_RETRACTED_POSITON, true));

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

    //operatorController.rightTrigger().onTrue(new VariableShootCommand(hopperSubsystem, driveSubsystem));
    //operatorController.x().onTrue(hopperSubsystem.simpleShootCommand());
    //operatorController.rightBumper().onTrue(climbCommand);
    //operatorController.leftBumper().onTrue(Commands.runOnce(climbCommand::cancel));
    //operatorController.a().onTrue(hopperSubsystem.intakeCommand());
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("test");
  }
}
