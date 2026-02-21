package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.LimelightHelpers;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.CANConstants.*;
import static frc.robot.Constants.DriveConstants.*;

public class DriveSubsystem extends SubsystemBase {
  private final Pigeon2 pigeon = new Pigeon2(PIGEON_ID);
  private final SwerveModule frontLeft = new SwerveModule(
      FRONT_LEFT_DRIVE_ID, FRONT_LEFT_TURN_ID, FRONT_LEFT_ABSOLUTE_ENCODER_ID, FRONT_LEFT_ANGULAR_OFFSET);
  private final SwerveModule frontRight = new SwerveModule(
      FRONT_RIGHT_DRIVE_ID, FRONT_RIGHT_TURN_ID, FRONT_RIGHT_ABSOLUTE_ENCODER_ID, FRONT_RIGHT_ANGULAR_OFFSET);
  private final SwerveModule rearLeft = new SwerveModule(
      REAR_LEFT_DRIVE_ID, REAR_LEFT_TURN_ID, REAR_LEFT_ABSOLUTE_ENCODER_ID, REAR_LEFT_ANGULAR_OFFSET);
  private final SwerveModule rearRight = new SwerveModule(
      REAR_RIGHT_DRIVE_ID, REAR_RIGHT_TURN_ID, REAR_RIGHT_ABSOLUTE_ENCODER_ID, REAR_RIGHT_ANGULAR_OFFSET);

  private SwerveModuleState[] moduleStates = DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds());
  private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
      frontLeft.getModulePosition(),
      frontRight.getModulePosition(),
      rearLeft.getModulePosition(),
      rearRight.getModulePosition()
  };

  private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
      DRIVE_KINEMATICS, new Rotation2d(pigeon.getYaw().getValue()), modulePositions, new Pose2d());

  public DriveSubsystem() {
    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
  }

  @Override
  public void periodic() {
    modulePositions[0] = frontLeft.getModulePosition();
    modulePositions[1] = frontRight.getModulePosition();
    modulePositions[2] = rearLeft.getModulePosition();
    modulePositions[3] = rearRight.getModulePosition();

    poseEstimator.update(new Rotation2d(getGyroscopeYaw()), modulePositions);

    LimelightHelpers.SetRobotOrientation(
        LIMELIGHT_HOSTNAME,
        poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
        pigeon.getAngularVelocityZDevice().getValue().in(DegreesPerSecond),
        0.0, 0.0, 0.0, 0.0);
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers
        .getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_HOSTNAME);
    poseEstimator.addVisionMeasurement(
        limelightMeasurement.pose,
        limelightMeasurement.timestampSeconds);
  }

  /**
   * Drive the robot using joystick inputs
   *
   * @param xSpeed        commanded velocity of the robot in the x-direction
   * @param ySpeed        commanded velocity of the robot in the y-direction
   * @param rot           commanded angular speed of the robot
   * @param fieldRelative whether the provided x and y speeds are field relative
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xSpeedDelivered = xSpeed * MAX_SPEED.in(MetersPerSecond);
    double ySpeedDelivered = ySpeed * MAX_SPEED.in(MetersPerSecond);
    double rotDelivered = rot * MAX_TURN_RATE.in(RadiansPerSecond);

    moduleStates = DRIVE_KINEMATICS.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered,
                new Rotation2d(getGyroscopeYaw()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        moduleStates, MAX_SPEED);
    setModuleStates(moduleStates);
  }

  /**
   * Drive the robot with robot relative {@link ChassisSpeeds}
   * 
   * @param speeds
   */
  public void drive(ChassisSpeeds speeds) {
    moduleStates = DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_SPEED);
    setModuleStates(moduleStates);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public Command setXCommand = Commands.runOnce(() -> {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }, this);

  public Pose2d getEstimatedRobotPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetEstimatedPose(Pose2d pose) {
    poseEstimator.resetPosition(new Rotation2d(getGyroscopeYaw()), modulePositions, pose);
  }

  public Angle getGyroscopeYaw() {
    return pigeon.getYaw().getValue();
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  private void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Reset every swerve modules encoders to the start state.
   */
  public void resetEncoders() {
    frontLeft.resetControllers();
    rearLeft.resetControllers();
    frontRight.resetControllers();
    rearRight.resetControllers();
  }

  public void zeroGyroscope() {
    pigeon.reset();
  }

  public ChassisSpeeds getChassisSpeeds() {
    SwerveModuleState[] states = new SwerveModuleState[] {
        frontLeft.getModuleState(),
        frontRight.getModuleState(),
        rearLeft.getModuleState(),
        rearRight.getModuleState()
    };
    ChassisSpeeds chassisSpeeds = DRIVE_KINEMATICS.toChassisSpeeds(states);
    return chassisSpeeds;
  }
}
