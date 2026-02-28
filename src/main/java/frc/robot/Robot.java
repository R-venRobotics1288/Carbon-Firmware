// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private GenericEntry m_FrontLeftEntry;
  private GenericEntry m_FrontRightEntry;
  private GenericEntry m_RearLeftEntry;
  private GenericEntry m_RearRightEntry;
  private GenericEntry m_JoystickX;
  private GenericEntry m_JoystickY;
  private GenericEntry m_JoystickRot;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_robotContainer.m_robotDrive.resetGyro();
    m_FrontLeftEntry = m_robotContainer.m_shuffle.shuffleTab
        .add("front left encoder radians",
            m_robotContainer.m_robotDrive.m_frontLeft.m_turningEncoder.getAbsolutePosition().getValueAsDouble())
        .getEntry();
    m_FrontRightEntry = m_robotContainer.m_shuffle.shuffleTab
        .add("front right encoder radians",
            m_robotContainer.m_robotDrive.m_frontRight.m_turningEncoder.getAbsolutePosition().getValueAsDouble())
        .getEntry();
    m_RearLeftEntry = m_robotContainer.m_shuffle.shuffleTab
        .add("rear left encoder radians",
            m_robotContainer.m_robotDrive.m_rearLeft.m_turningEncoder.getAbsolutePosition().getValueAsDouble())
        .getEntry();
    m_RearRightEntry = m_robotContainer.m_shuffle.shuffleTab
        .add("rear right encoder radians",
            m_robotContainer.m_robotDrive.m_rearRight.m_turningEncoder.getAbsolutePosition().getValueAsDouble())
        .getEntry();
    m_JoystickX = m_robotContainer.m_shuffle.shuffleTab
        .add("Joystick X value",
          m_robotContainer.m_driverController.getLeftX())
        .getEntry();
    m_JoystickY = m_robotContainer.m_shuffle.shuffleTab
        .add("Joystick Y value",
          m_robotContainer.m_driverController.getLeftY())
        .getEntry();
    m_JoystickRot = m_robotContainer.m_shuffle.shuffleTab
      .add("Joystick Rotation Value",
        m_robotContainer.m_driverController.getRightX())
      .getEntry();
    }
   
      /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // tmpentry.setDouble(m_robotContainer.m_robotDrive.m_frontLeft.m_turningEncoder.getAbsolutePosition().getValueAsDouble());
    
    m_robotContainer.refresh_shuffleboard();
    m_FrontLeftEntry.setDouble(m_robotContainer.m_robotDrive.m_frontLeft.getAbsoluteEncoderRad()); //Returns in Radians
    m_FrontRightEntry.setDouble(m_robotContainer.m_robotDrive.m_frontRight.getAbsoluteEncoderRad());
    m_RearLeftEntry.setDouble(m_robotContainer.m_robotDrive.m_rearLeft.getAbsoluteEncoderRad());
    m_RearRightEntry.setDouble(m_robotContainer.m_robotDrive.m_rearRight.getAbsoluteEncoderRad());
    m_JoystickX.setDouble(m_robotContainer.m_driverController.getLeftX());
    m_JoystickY.setDouble(m_robotContainer.m_driverController.getLeftY());
    m_JoystickRot.setDouble(m_robotContainer.m_driverController.getRightX());
    //m_robotContainer.printLimeLight();
    m_robotContainer.updateOdometry();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
