package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShuffleValues;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class GoToRelativePose extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final XboxController m_controller;
    private final double m_relativeX;
    private final double m_relativeY;

    private Pose2d m_targetPose;

    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_thetaController;

    /**
     * Creates a new GoToRelativePose command.
     *
     * @param driveSubsystem The drive subsystem to use.
     * @param controller The driver's controller, used for cancellation.
     * @param relativeX The desired relative movement in the X direction (forward/backward).
     * @param relativeY The desired relative movement in the Y direction (left/right).
     */
    public GoToRelativePose(DriveSubsystem driveSubsystem, XboxController controller, double relativeX, double relativeY) {
        this.m_driveSubsystem = driveSubsystem;
        this.m_controller = controller;
        this.m_relativeX = relativeX;
        this.m_relativeY = relativeY;

        m_xController = new PIDController(AutoConstants.kPXController, 0, 0);
        m_yController = new PIDController(AutoConstants.kPYController, 0, 0);
        m_thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize() {
        // Get the current pose and calculate the target pose by applying the relative transform.
        Pose2d currentPose = m_driveSubsystem.getPose();
        Transform2d relativeTransform = new Transform2d(new Translation2d(m_relativeX, m_relativeY), new Rotation2d());
        m_targetPose = currentPose.plus(relativeTransform);

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
        xSpeed = MathUtil.clamp(xSpeed, -ShuffleValues.kMaxSpeedMetersPerSecond, ShuffleValues.kMaxSpeedMetersPerSecond);
        ySpeed = MathUtil.clamp(ySpeed, -ShuffleValues.kMaxSpeedMetersPerSecond, ShuffleValues.kMaxSpeedMetersPerSecond);
        thetaSpeed = MathUtil.clamp(thetaSpeed, -ShuffleValues.kMaxAngularSpeed, ShuffleValues.kMaxAngularSpeed);

        // Create ChassisSpeeds from the calculated speeds and pass to the subsystem
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed, m_driveSubsystem.getPose().getRotation());
        m_driveSubsystem.setChassisSpeeds(chassisSpeeds);
    }

    @Override
    public boolean isFinished() {
        // The command is finished when the robot is at the setpoint.
        return m_xController.atSetpoint() && m_yController.atSetpoint() && m_thetaController.atSetpoint();
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