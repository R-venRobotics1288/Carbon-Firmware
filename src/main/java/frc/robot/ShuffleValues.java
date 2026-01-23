package frc.robot;
import java.util.Map;


public class ShuffleValues {
    public static boolean SHUFFLE_MANAGER_ENABLED = true;

  public static boolean kfieldRelative = false;
  // Driving Parameters - Note that these are not the maximum capable speeds of
  // the robot, rather the allowed maximum speeds
  public static double kMaxSpeedMetersPerSecond = 0.06;
  public static double kMaxAngularSpeed = 0.1 * Math.PI; // radians per second
  public static double kDriveDeadband = 0.075;
  public static double slewrate_translation = 0.5;
  public static double slewrate_rotation = 0.3;
}