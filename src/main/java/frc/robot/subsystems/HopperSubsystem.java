package frc.robot.subsystems;

import static frc.robot.Constants.CANConstants.*;
import static frc.robot.Constants.HopperConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
  private final SparkFlex shooterMotor;
  private final SparkFlex intakeMotor;
  private final InterpolatingDoubleTreeMap shooterPowerMap; // key Meters value Power %

  public HopperSubsystem() {
    shooterMotor = new SparkFlex(SHOOTER_ID, MotorType.kBrushless);
    intakeMotor = new SparkFlex(INTAKE_ID, MotorType.kBrushless);

    shooterMotor.configure(SHOOTER_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotor.configure(INTAKE_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    shooterPowerMap = new InterpolatingDoubleTreeMap();
    shooterPowerMap.put(1.0, 0.6); // TODO: placeholders, test and find legit values
    shooterPowerMap.put(2.0, 0.7);
    shooterPowerMap.put(3.0, 0.8);
  }

  public void setMotorSpeeds(double intakeSpeed, double shooterSpeed) {
    intakeMotor.set(intakeSpeed);
    shooterMotor.set(shooterSpeed);
  }

  public void stopShooter() {
    shooterMotor.stopMotor();
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  public double getPower(double distance) {
    return shooterPowerMap.get(distance);
  }

  public Command intakeCommand() {
    return Commands.startEnd(() -> {
      setMotorSpeeds(INTAKE_SPEED, 0);
    }, () -> {
      stopIntake();
    });
  }

  public Command simpleShootCommand() {
    return Commands.startEnd(() -> {
      setMotorSpeeds(-INTAKE_SPEED, SHOOTER_SPEED);
    }, () -> {
      stopIntake();
      stopShooter();
    }, this);
  }
}
