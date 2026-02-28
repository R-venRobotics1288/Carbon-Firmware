// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 1.25;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(18);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24);
    public static final double kDriveDeadband = 0.09;
    // Distance between front and rear wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
      
      // Angular offsets of the modules relative to the chassis in radians
      public static final double kFrontLeftChassisAngularOffset = Math.PI; //+ (0.3583984375 * 2 * Math.PI);
      public static final double kFrontRightChassisAngularOffset = 0; //+ (0.623779296875 * 2 * Math.PI);
      public static final double kRearLeftChassisAngularOffset = 0; //+ (0.430419921875 * 2 * Math.PI);
      public static final double kRearRightChassisAngularOffset = Math.PI; //+ (0.28466796875 * 2 * Math.PI);

      // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearLeftDrivingCanId = 7;
    public static final int kRearRightDrivingCanId = 10; 

    public static final int kFrontLeftTurningCanId = 15;
    public static final int kFrontRightTurningCanId = 6;
    public static final int kRearLeftTurningCanId = 9;
    public static final int kRearRightTurningCanId = 12;

    //Encoder CAN ID values
    public static final int kFrontLeftAbsoluteEncoderCanId = 14;
    public static final int kFrontRightAbsoluteEncoderCanId = 5;
    public static final int kRearLeftAbsoluteEncoderCanId = 8;
    public static final int kRearRightAbsoluteEncoderCanId = 11;

    //Gyro CAN ID
    public static final int kGyroCanID = 1;

    public static final boolean kGyroReversed = false;
    public static final boolean kfieldRelative = true;

    public static final SlewRateLimiter translationfilterx = new SlewRateLimiter(.75);
    public static final SlewRateLimiter translationfiltery = new SlewRateLimiter(.75);
    public static final SlewRateLimiter rotationfilter = new SlewRateLimiter(Math.PI);
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0952;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    //public static final double kDrivingMotorReduction = 5.14;
    public static final double kDrivingMotorReduction = 6.75;
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double drivingFactor = (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }

  public static final class HopperConstants {
    public static final int kShooterCANID = 17;
    public static final int kIntakeCANID = 16;

    public static final double kIntakeMotorSpeed = -0.30;
    public static final double kShooterMotorSpeed = 0.80;

    public static final Translation2d kBlueHubPosition = new Translation2d(4.625, 4.035);
    public static final Translation2d kRedHubPosition = new Translation2d(11.915, 4.035);
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