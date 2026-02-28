package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class Constants {

  public static final class HIDConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final double DRIVER_CONTROLLER_DEADBAND = 0.09;
    public static final double TRANSLATION_SLEW_LIMIT = 2;
    public static final double ROTATION_SLEW_LIMIT = 1;
  }

  public static final class CANConstants {
    public static final int PIGEON_ID = 1;

    public static final int FRONT_LEFT_DRIVE_ID = 13;
    public static final int FRONT_RIGHT_DRIVE_ID = 4;
    public static final int REAR_LEFT_DRIVE_ID = 7;
    public static final int REAR_RIGHT_DRIVE_ID = 10;

    public static final int FRONT_LEFT_TURN_ID = 15;
    public static final int FRONT_RIGHT_TURN_ID = 6;
    public static final int REAR_LEFT_TURN_ID = 9;
    public static final int REAR_RIGHT_TURN_ID = 12;

    public static final int FRONT_LEFT_ABSOLUTE_ENCODER_ID = 14;
    public static final int FRONT_RIGHT_ABSOLUTE_ENCODER_ID = 5;
    public static final int REAR_LEFT_ABSOLUTE_ENCODER_ID = 8;
    public static final int REAR_RIGHT_ABSOLUTE_ENCODER_ID = 11;

    public static final int INTAKE_ID = 16;
    public static final int SHOOTER_ID = 17;

    public static final int LEFT_CLIMBER_ID = 18;
    public static final int RIGHT_CLIMBER_ID = 19; // FIXME: may not use both motors, liason with mechanical
  }

  public static final class DriveConstants {
    public static final String LIMELIGHT_HOSTNAME = "limelight";
    // Actual max speed and turn rate are set on the Dashboard, see Dashboard.java
    public static final LinearVelocity DEFAULT_MAX_SPEED = MetersPerSecond.of(5);
    public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(3);

    public static final AngularVelocity DEFAULT_MAX_TURN_RATE = RevolutionsPerSecond.of(0.85);
    public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RadiansPerSecondPerSecond.of(35);

    public static final Distance TRACK_WIDTH = Inches.of(18);
    public static final Distance WHEEL_BASE = Inches.of(24);
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE.div(2), TRACK_WIDTH.div(2)),
        new Translation2d(WHEEL_BASE.div(2), TRACK_WIDTH.div(2).unaryMinus()),
        new Translation2d(WHEEL_BASE.div(2).unaryMinus(), TRACK_WIDTH.div(2)),
        new Translation2d(WHEEL_BASE.div(2).unaryMinus(), TRACK_WIDTH.div(2).unaryMinus()));

    public static final Angle FRONT_LEFT_ANGULAR_OFFSET = Radians.of(Math.PI);
    public static final Angle FRONT_RIGHT_ANGULAR_OFFSET = Radians.of(0);
    public static final Angle REAR_LEFT_ANGULAR_OFFSET = Radians.of(0);
    public static final Angle REAR_RIGHT_ANGULAR_OFFSET = Radians.of(Math.PI);

    public static final class ModuleConstants {
      public static final double DRIVE_MOTOR_REDUCTION = 6.75;
      public static final double TURNING_MOTOR_REDUCTION = 12.8;
      public static final AngularVelocity NEO_VORTEX_FREE_SPEED = RPM.of(6784);
      public static final Distance WHEEL_DIAMETER = Inches.of(3);

      public static final double DRIVING_FACTOR = WHEEL_DIAMETER.in(Meters) * Math.PI / DRIVE_MOTOR_REDUCTION;
      public static final SparkBaseConfig DRIVE_MOTOR_CONFIG = SparkFlexConfig.Presets.REV_Vortex;
      static {
        DRIVE_MOTOR_CONFIG
            .idleMode(IdleMode.kBrake);
        DRIVE_MOTOR_CONFIG.encoder
            .positionConversionFactor(DRIVING_FACTOR) // conversion from rotations to meters
            .velocityConversionFactor(DRIVING_FACTOR / 60.0); // conversion from RPM to m/s
        DRIVE_MOTOR_CONFIG.closedLoop
            .outputRange(-1, 1)
            .pid(0.5, 0, 0);
        DRIVE_MOTOR_CONFIG.closedLoop.maxMotion
            .maxAcceleration(MAX_ACCELERATION.in(MetersPerSecondPerSecond))
            .allowedProfileError(0.05); // 5 cm/s maximum velocity error (1% of a 5m/s max)
      }

      public static final SparkBaseConfig TURN_MOTOR_CONFIG = SparkFlexConfig.Presets.REV_Vortex;
      // Note: no closed loop on the turning motor since we use an non-native CANcoder
      static {
        TURN_MOTOR_CONFIG.idleMode(IdleMode.kBrake);
      }
      public static final double TURN_MOTOR_P = 0.15;
      public static final double TURN_MOTOR_I = 0;
      public static final double TURN_MOTOR_D = 0;
    }
  }

  public static final class AutonomousConstants {
    public static final double PP_DRIVE_PID_P = 4.8; // TODO: Tune PID for autonomous path following.
    public static final double PP_DRIVE_PID_I = 0;
    public static final double PP_DRIVE_PID_D = 0.15;
    public static final double PP_TURN_PID_P = 4.3;
    public static final double PP_TURN_PID_I = 0;
    public static final double PP_TURN_PID_D = 0.15;
  }

  public static final class HopperConstants {
    // speeds are in motor power %, we could also use the closed loop controller to
    // setpoint to a desired RPM velocity
    public static final double INTAKE_SPEED = -0.30;
    public static final double SHOOTER_SPEED = 0.80;

    public static final SparkBaseConfig INTAKE_MOTOR_CONFIG = SparkFlexConfig.Presets.REV_Vortex;
    static {
      INTAKE_MOTOR_CONFIG.idleMode(IdleMode.kCoast);
    }
    public static final SparkBaseConfig SHOOTER_MOTOR_CONFIG = SparkFlexConfig.Presets.REV_Vortex;
    static {
      SHOOTER_MOTOR_CONFIG.idleMode(IdleMode.kCoast);
    }

    public static final Translation2d BLUE_HUB_POSITION = new Translation2d(4.625, 4.035);
    public static final Translation2d RED_HUB_POSITION = new Translation2d(11.915, 4.035);
  }

  public static final class ClimberConstants {
    public static final double CLIMBER_GEAR_RATIO = 1.0; // FIXME: change later, check with mechanical
    public static final SparkBaseConfig CLIMBER_MOTOR_CONFIG = SparkMaxConfig.Presets.REV_Vortex;
    static {
      CLIMBER_MOTOR_CONFIG
          .idleMode(IdleMode.kBrake);
      CLIMBER_MOTOR_CONFIG.encoder
          .positionConversionFactor(CLIMBER_GEAR_RATIO);
      CLIMBER_MOTOR_CONFIG.closedLoop
          .outputRange(-0.4, 0.4)
          .pid(1.0, 0.0, 0.0); // TODO: tune closed loop PID, output range, and maxMotion parameters
      CLIMBER_MOTOR_CONFIG.closedLoop.maxMotion
          .cruiseVelocity(1.0) // RPS
          .maxAcceleration(1.0); // RPSPS
    }

    public static final Angle CLIMBER_ZERO_POSITION = Rotations.of(0); // TODO: set positions
    public static final Angle CLIMBER_RETRACTED_POSITON = Rotations.of(1);
    public static final Angle CLIMBER_POSITION_ONE = Rotations.of(12);
  }
}