// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.math.filter.SlewRateLimiter;

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

  public static boolean SHUFFLE_MANAGER_ENABLED;

  public static boolean kfieldRelative;
  // Driving Parameters - Note that these are not the maximum capable speeds of
  // the robot, rather the allowed maximum speeds
  public static double kMaxSpeedMetersPerSecond = 0.06;
  public static double kMaxAngularSpeed = 0.1 * Math.PI; // radians per second
  public static double kDriveDeadband = 0.075;
  public static double slewrate_translation = 0.5;
  public static double slewrate_rotation = 0.3;

  public ShuffleboardTab shuffleTab = Shuffleboard.getTab("tooning");

  private GenericEntry slew = shuffleTab.addPersistent("xy slew",
      slewrate_translation)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 1,
          "max", 50))
      .getEntry();
  private GenericEntry maxSpeed = shuffleTab.addPersistent("max speed",
      kMaxSpeedMetersPerSecond)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 1,
          "max", 75))
      .getEntry();
  private GenericEntry maxRot = shuffleTab.addPersistent("max rot per s",
      kMaxAngularSpeed)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 1,
          "max", 30))
      .getEntry();
  private GenericEntry rotSlew = shuffleTab.addPersistent("rotation slew",
      kMaxAngularSpeed)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 1,
          "max", 30))
      .getEntry();
  private GenericEntry fieldRelative = shuffleTab.addPersistent("field relative", kfieldRelative)
      .withWidget(BuiltInWidgets.kToggleButton).getEntry();

  public void refreshValues(SlewRateLimiter x, SlewRateLimiter y, SlewRateLimiter rotation) {
    if (SHUFFLE_MANAGER_ENABLED) {
      if (kMaxSpeedMetersPerSecond != maxSpeed.getDouble(kMaxSpeedMetersPerSecond)) {
        kMaxSpeedMetersPerSecond = maxSpeed.getDouble(kMaxSpeedMetersPerSecond);
        System.out.println("changed speed");
      }
      if (kMaxAngularSpeed != maxRot.getDouble(kMaxAngularSpeed)) {
        kMaxAngularSpeed = maxRot.getDouble(kMaxAngularSpeed);
        kMaxAngularSpeed = maxRot.getDouble(kMaxAngularSpeed) * 2 * Math.PI;
      }
      if (slewrate_translation != slew.getDouble(slewrate_translation)) {
        slewrate_translation = slew.getDouble(slewrate_translation);
        x = new SlewRateLimiter(slewrate_translation);
        y = new SlewRateLimiter(slewrate_translation);
      }
      if (slewrate_rotation != rotSlew.getDouble(slewrate_rotation)) {
        slewrate_rotation = rotSlew.getDouble(slewrate_rotation);
        rotation = new SlewRateLimiter(rotSlew.getDouble(slewrate_rotation));
      }
      if (slewrate_rotation != rotSlew.getDouble(slewrate_rotation)) {
        slewrate_rotation = rotSlew.getDouble(slewrate_rotation);
        rotation = new SlewRateLimiter(rotSlew.getDouble(slewrate_rotation));
      }
      if (kfieldRelative != fieldRelative.getBoolean(kfieldRelative)) {
        kfieldRelative = fieldRelative.getBoolean(kfieldRelative);
      }
    }
  }
  public void changeFieldRelative(Boolean input) {
    kfieldRelative = input;
    fieldRelative.setBoolean(input);
  }
}
