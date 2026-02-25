package frc.robot.utilities;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

public class Dashboard {
  private final DoubleEntry speedLimitEntry; // Meters per Second
  private final DoubleEntry rotLimitEntry; // Radians per Second

  public Dashboard() {
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTable nt = instance.getTable("carbon");

    speedLimitEntry = nt.getDoubleTopic("speedlimit").getEntry(DEFAULT_MAX_SPEED.in(MetersPerSecond));
    speedLimitEntry.set(DEFAULT_MAX_SPEED.in(MetersPerSecond));

    rotLimitEntry = nt.getDoubleTopic("rotlimit").getEntry(DEFAULT_MAX_TURN_RATE.in(RadiansPerSecond));
    rotLimitEntry.set(DEFAULT_MAX_TURN_RATE.in(RadiansPerSecond));
  }

  public LinearVelocity getSpeedLimit() {
    return MetersPerSecond.of(speedLimitEntry.get());
  }

  public AngularVelocity getRotLimit() {
    return RadiansPerSecond.of(rotLimitEntry.get());
  }
}
