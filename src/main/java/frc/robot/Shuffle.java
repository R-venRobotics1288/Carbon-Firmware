// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

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
public final class Shuffle {
  public static boolean fieldRelative;
  // Driving Parameters - Note that these are not the maximum capable speeds of
  // the robot, rather the allowed maximum speeds
  public static final double kMaxSpeedMetersPerSecond = 0.06;
  public static final double kMaxAngularSpeed = 0.03 * Math.PI; // radians per second
  public static final double kDriveDeadband = 0.075;
  public static double slewrate_translation = 0.5;
  public static double slewrate_rotation = 0.5;

  public ShuffleboardTab shuffleTab = Shuffleboard.getTab("tooning");
  /*
   * private GenericEntry slew = shuffleTab.addPersistent("xy slew",
   * BASE_SLEW_RATE)
   * .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 1,
   * "max", 50)).getEntry();
   * private GenericEntry maxSpeed = shuffleTab.addPersistent("max speed",
   * kMaxSpeedMetersPerSecond)
   * .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 1,
   * "max", 75)).getEntry();
   * private GenericEntry maxRot = shuffleTab.addPersistent("max rot per s",
   * kMaxAngularSpeed)
   * .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 1,
   * "max", 30)).getEntry();
   * private GenericEntry rotSlew = shuffleTab.addPersistent("rotation slew",
   * MAX_ROBOT_ROTATIONS_PER_SECOND)
   * .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 1,
   * "max", 30)).getEntry();
   */
}
