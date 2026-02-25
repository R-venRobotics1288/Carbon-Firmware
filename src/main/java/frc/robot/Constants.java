package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class Constants {

  public static final class HIDConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final double DRIVER_CONTROLLER_DEADBAND = 0.05;
    public static final double TRANSLATION_SLEW_LIMIT = 1; // TODO: Tune both slew rate limits.
    public static final double ROTATION_SLEW_LIMIT = 0.5;
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
  }

  public static final class DriveConstants {
    public static final String LIMELIGHT_HOSTNAME = "limelight";
    // Actual max speed and turn rate are set on the Dashboard, see Dashboard.java
    public static final LinearVelocity DEFAULT_MAX_SPEED = MetersPerSecond.of(5);
    public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(2.5);

    public static final AngularVelocity DEFAULT_MAX_TURN_RATE = RevolutionsPerSecond.of(0.85);
    public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RadiansPerSecondPerSecond.of(30);

    public static final Distance TRACK_WIDTH = Inches.of(18);
    public static final Distance WHEEL_BASE = Inches.of(24);
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE.div(2), TRACK_WIDTH.div(2)),
        new Translation2d(WHEEL_BASE.div(2), TRACK_WIDTH.div(2).unaryMinus()),
        new Translation2d(WHEEL_BASE.div(2).unaryMinus(), TRACK_WIDTH.div(2)),
        new Translation2d(WHEEL_BASE.div(2).unaryMinus(), TRACK_WIDTH.div(2).unaryMinus()));

    public static final double FRONT_LEFT_ANGULAR_OFFSET = Math.PI;
    public static final double FRONT_RIGHT_ANGULAR_OFFSET = 0;
    public static final double REAR_LEFT_ANGULAR_OFFSET = 0;
    public static final double REAR_RIGHT_ANGULAR_OFFSET = Math.PI;

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
            .pid(0, 0, 0); // TODO: Tune kS and PID for drive.
        DRIVE_MOTOR_CONFIG.closedLoop.feedForward
            .kS(0);
        DRIVE_MOTOR_CONFIG.closedLoop.maxMotion
            .maxAcceleration(MAX_ACCELERATION.in(MetersPerSecondPerSecond))
            .allowedProfileError(0.1); // 10 cm/s maximum velocity error (2% of a 5m/s max)
      }

      public static final SparkBaseConfig TURN_MOTOR_CONFIG = SparkFlexConfig.Presets.REV_Vortex;
      // Note: no closed loop on the turning motor since we use an non-native CANcoder
      static {
        TURN_MOTOR_CONFIG.idleMode(IdleMode.kBrake);
      }
      public static final double TURN_MOTOR_P = 0; // TODO: Tune kS and PID for turning.
      public static final double TURN_MOTOR_I = 0;
      public static final double TURN_MOTOR_D = 0;
      public static final double TURN_MOTOR_KS = 0;
    }
  }

  public static final class AutonomousConstants {
    public static final double PP_DRIVE_PID_P = 0; // TODO: Tune PID for autonomous path following.
    public static final double PP_DRIVE_PID_I = 0;
    public static final double PP_DRIVE_PID_D = 0;
    public static final double PP_TURN_PID_P = 0;
    public static final double PP_TURN_PID_I = 0;
    public static final double PP_TURN_PID_D = 0;
  }

  public static final class ClimberConstants {
    public static final int kLeftCANId = 18;
    public static final int kRightCANId = 19; //may not use one motor

    public static final double kGearRatio = 1.0; //change later

    public static final double kMaxMotorSpeed = 1.0;

    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kPositionTolerance = 0.05; //in meters

    public static final double kDesiredPosZero = 0.0;
    public static final double kRetractedDesiredPos = 0.05;
    public static final double kDesiredPosOne = 0.3; //in meters, should be changed
  }
}