package frc.robot.commands;

import java.io.Console;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;

public class VariableShootCommand extends Command {
    private HopperSubsystem m_hopperSubsystem;
    private DriveSubsystem m_driveSubsystem;
    private double m_distance;
    private double m_power;
    private final Translation2d kHubPosition;
    private PIDController thetaController;

    public VariableShootCommand(HopperSubsystem hopperSubsystem, DriveSubsystem driveSubsystem) {
        m_hopperSubsystem = hopperSubsystem;
        m_driveSubsystem = driveSubsystem;
        kHubPosition = DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? HopperConstants.kRedHubPosition : HopperConstants.kBlueHubPosition;
        addRequirements(hopperSubsystem);
    }

    @Override
    public void initialize() {
        thetaController = new PIDController(0.075, 0, 0.0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(2 * Math.PI * 5/360);
    }

    @Override
    public void execute() {
        Pose2d robotPose = m_driveSubsystem.getPose();
        double angle = Math.atan2(kHubPosition.getY() - robotPose.getY(), kHubPosition.getX() - robotPose.getX());
        thetaController.setSetpoint(angle);
        double angleOverride = (thetaController.calculate((m_driveSubsystem.getHeading() / 360) * 2 * Math.PI))*2*Math.PI;
        m_driveSubsystem.setYawOverride(angleOverride);
        //System.out.println(angleOverride);
        m_distance = robotPose.getTranslation().getDistance(kHubPosition);
        System.out.println(m_distance);
        // m_power = m_hopperSubsystem.getFlywheelPower(m_distance);
        // m_hopperSubsystem.setMotorSpeed(m_power, HopperConstants.kShooterFeederMotorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_hopperSubsystem.stopMotors();
        m_driveSubsystem.clearYawOverride();
    }
}
