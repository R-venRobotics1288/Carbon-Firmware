package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class GoToPose extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final XboxController m_controller;
    private final Pose2d m_targetPose;

    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_thetaController;

    /**
     * Creates a new GoToPose command.
     *
     * @param driveSubsystem The drive subsystem to use.
     * @param controller The driver's controller, used for cancellation.
     * @param targetPose The absolute field-relative pose to drive to.
     */
    public GoToPose(DriveSubsystem driveSubsystem, XboxController controller, Pose2d targetPose) {
        this.m_driveSubsystem = driveSubsystem;
        this.m_controller = controller;
        this.m_targetPose = targetPose;

        m_xController = new PIDController(AutoConstants.kPXController, 0, 0);
        m_yController = new PIDController(AutoConstants.kPYController, 0, 0);
        m_thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize() {
        // Set the tolerance for the PID controllers
        m_xController.setTolerance(0.05); // 5 cm
        m_yController.setTolerance(0.05); // 5 cm
        m_thetaController.setTolerance(Math.toRadians(2)); // 2 degrees
    }

    @Override
    public void execute() {
        Pose2d currentPose = m_driveSubsystem.getPose();

        // Calculate the required speeds to reach the target pose
        double xSpeed = m_xController.calculate(currentPose.getX(), m_targetPose.getX());
        double ySpeed = m_yController.calculate(currentPose.getY(), m_targetPose.getY());
        double thetaSpeed = m_thetaController.calculate(currentPose.getRotation().getRadians(), m_targetPose.getRotation().getRadians());

        // Clamp the output of the PID controllers to the robot's maximum speeds
        xSpeed = MathUtil.clamp(xSpeed, -AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxSpeedMetersPerSecond);
        ySpeed = MathUtil.clamp(ySpeed, -AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxSpeedMetersPerSecond);
        thetaSpeed = MathUtil.clamp(thetaSpeed, -AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularSpeedRadiansPerSecond);

        // Create ChassisSpeeds from the calculated speeds and pass to the subsystem
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed, m_driveSubsystem.getPose().getRotation());
        m_driveSubsystem.setChassisSpeeds(chassisSpeeds);
    }

    @Override
    public boolean isFinished() {
        // The command is finished when the robot is at the setpoint or the driver moves the sticks.
        boolean atTarget = m_xController.atSetpoint() && m_yController.atSetpoint() && m_thetaController.atSetpoint();
        return atTarget || driverIsMoving();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends or is interrupted.
        m_driveSubsystem.stop();
    }

    private boolean driverIsMoving() {
        // Check if the driver is trying to take back control
        return Math.abs(m_controller.getLeftX()) > DriveConstants.kDriveDeadband ||
               Math.abs(m_controller.getLeftY()) > DriveConstants.kDriveDeadband ||
               Math.abs(m_controller.getRightX()) > DriveConstants.kDriveDeadband;
    }
}