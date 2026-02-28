package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.CANConstants.*;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;

  public ClimberSubsystem() {
    leftMotor = new SparkMax(LEFT_CLIMBER_ID, MotorType.kBrushless);
    leftMotor.configureAsync(CLIMBER_MOTOR_CONFIG, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    rightMotor = new SparkMax(RIGHT_CLIMBER_ID, MotorType.kBrushless);
    rightMotor.configureAsync(CLIMBER_MOTOR_CONFIG, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  /**
   * Climbs to the specified position.
   * 
   * @param desiredPosition desired position to climb to
   * @param useLeftMotor    true to use left motor, if false, uses right motor
   * @return the Command object
   */
  public Command climbCommand(Angle desiredPosition, boolean useLeftMotor) {
    if (useLeftMotor) {
      return Commands.startEnd(() -> {
        leftMotor.getClosedLoopController().setSetpoint(
            desiredPosition.in(Rotations),
            ControlType.kMAXMotionPositionControl);
      }, () -> {
        leftMotor.stopMotor();
      }, this).until(leftMotor.getClosedLoopController()::isAtSetpoint);
    } else {
      return Commands.startEnd(() -> {
        rightMotor.getClosedLoopController().setSetpoint(
            desiredPosition.in(Rotations),
            ControlType.kMAXMotionPositionControl);
      }, () -> {
        rightMotor.stopMotor();
      }, this).until(rightMotor.getClosedLoopController()::isAtSetpoint);
    }
  }
}