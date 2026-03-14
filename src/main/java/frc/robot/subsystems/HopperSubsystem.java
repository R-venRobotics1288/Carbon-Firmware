package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.HopperConfigs;
import frc.robot.Constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase {

    private final SparkFlex m_feederMotor;
    private final SparkFlex m_flywheelMotor;
    private final InterpolatingDoubleTreeMap shooterFlywheelPower;

    public HopperSubsystem() {
        m_feederMotor = new SparkFlex(HopperConstants.kShooterCANID, MotorType.kBrushless);
        m_flywheelMotor = new SparkFlex(HopperConstants.kFlywheelCANID, MotorType.kBrushless);

        m_feederMotor.configure(HopperConfigs.shooterConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_flywheelMotor.configure(HopperConfigs.intakeConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        
        shooterFlywheelPower = new InterpolatingDoubleTreeMap();
        shooterFlywheelPower.put(1.0, 0.6);
        shooterFlywheelPower.put(2.0, 0.7);
        shooterFlywheelPower.put(3.0, 0.8);

        setDefaultCommand(
        runOnce(
                () -> {
                  m_flywheelMotor.disable();
                  m_feederMotor.disable();
                })
            .andThen(run(() -> {}))
            .withName("Idle"));

    }

    public Command setMotorSpeed(double flywheelSpeed, double feederMotorSpeed, double delay) {
        return Commands.sequence(
            runOnce(() -> m_flywheelMotor.set(flywheelSpeed)),
            Commands.waitSeconds(delay),
            runOnce(() -> m_feederMotor.set(feederMotorSpeed))
        );
    }

    public Command shootCommand(double flywheelSpeed, double feederMotorSpeed, double delay) {
    return Commands.parallel(
            // Run the shooter flywheel at the desired setpoint using feedforward and feedback
            run(
                () -> {
                  m_flywheelMotor.set(flywheelSpeed);
                }),

            // Wait until the shooter has reached the setpoint, and then run the feeder
            Commands.waitSeconds(delay).andThen(() -> m_feederMotor.set(feederMotorSpeed)))
        .withName("Shoot");
  }

    public void stopMotors() {
        m_flywheelMotor.set(0);
        m_feederMotor.set(0);
        m_flywheelMotor.stopMotor();
        m_feederMotor.stopMotor();
    }

    public double getFlywheelPower(double distance) {
        return shooterFlywheelPower.get(distance);
    }

}
