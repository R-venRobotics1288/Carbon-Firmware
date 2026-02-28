package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.DriveConstants.ModuleConstants.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;

/**
 * Represents one physical Swerve Module.
 */
public class SwerveModule {
  private final SparkFlex drivingMotor;
  private final RelativeEncoder drivingEncoder;

  private final SparkFlex turningMotor;
  private final ProfiledPIDController turningController = new ProfiledPIDController(
      TURN_MOTOR_P,
      TURN_MOTOR_I,
      TURN_MOTOR_D,
      // Could split DEFAULT_MAX_TURN_RATE into a different constant, if divergent
      // behvaiour desired
      new Constraints(DEFAULT_MAX_TURN_RATE.in(RPM), MAX_ANGULAR_ACCELERATION.in(RPM.per(Second))));
  private final CANcoder turningEncoder;
  private final double chassisAngularOffset;

  private SwerveModuleState currentDesiredState = new SwerveModuleState();

  public SwerveModule(int drivingMotorID, int turningMotorID, int absoluteEncoderID, double chassisAngularOffset) {
    drivingMotor = new SparkFlex(drivingMotorID, MotorType.kBrushless);
    drivingMotor.configureAsync(DRIVE_MOTOR_CONFIG, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    drivingEncoder = drivingMotor.getEncoder();
    drivingEncoder.setPosition(0);

    turningMotor = new SparkFlex(turningMotorID, MotorType.kBrushless);
    turningMotor.configureAsync(TURN_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turningController.enableContinuousInput(-1.0, 1.0);
    turningController.setTolerance(0.05);
    turningEncoder = new CANcoder(absoluteEncoderID);
    currentDesiredState.angle = new Rotation2d(getAbsoluteEncoder());

    this.chassisAngularOffset = chassisAngularOffset;
  }

  public Angle getAbsoluteEncoder() {
    return turningEncoder.getAbsolutePosition().getValue();
  }

  public void resetControllers() {
    drivingEncoder.setPosition(0);
    turningController.reset(getAbsoluteEncoder().in(Rotations));
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(
        drivingEncoder.getPosition(),
        new Rotation2d(getAbsoluteEncoder()).minus(Rotation2d.fromRadians(chassisAngularOffset)));
  }

  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(
        drivingEncoder.getVelocity(),
        new Rotation2d(getAbsoluteEncoder()).minus(Rotation2d.fromRadians(chassisAngularOffset)));
  }

  public void setCurrentDesiredState(SwerveModuleState desiredState) {
    currentDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    currentDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));
    currentDesiredState.optimize(new Rotation2d(getAbsoluteEncoder()));

    drivingMotor.getClosedLoopController().setSetpoint(
        currentDesiredState.speedMetersPerSecond,
        ControlType.kMAXMotionVelocityControl);

    turningMotor.set(turningController.calculate(
        getAbsoluteEncoder().in(Rotations),
        currentDesiredState.angle.getRotations()));
  }
}
