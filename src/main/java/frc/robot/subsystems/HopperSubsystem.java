package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Configs.HopperConfigs;
import frc.robot.Constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase {

    private final SparkFlex m_feederMotor;
    private final SparkFlex m_flywheelMotor;
    private final InterpolatingDoubleTreeMap shooterFlywheelPower;

    public HopperSubsystem() {
        m_feederMotor = new SparkFlex(HopperConstants.kShooterCANID, MotorType.kBrushless);
        m_flywheelMotor = new SparkFlex(HopperConstants.kIntakeFlywheelCANID, MotorType.kBrushless);

        m_feederMotor.configure(HopperConfigs.shooterConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_flywheelMotor.configure(HopperConfigs.intakeConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        
        shooterFlywheelPower = new InterpolatingDoubleTreeMap();
        shooterFlywheelPower.put(1.0, 0.6);
        shooterFlywheelPower.put(2.0, 0.7);
        shooterFlywheelPower.put(3.0, 0.8);
    }

    public Command setMotorSpeed(double flywheelSpeed, double feederMotorSpeed) {
        return Commands.runEnd(() -> {
            m_flywheelMotor.set(flywheelSpeed);
            new WaitCommand(1);
            m_feederMotor.set(feederMotorSpeed);
            }, () -> {
            m_flywheelMotor.set(0);
            m_feederMotor.set(0);
            }, this);
    }

    public void stopMotors() {
        m_flywheelMotor.stopMotor();
        m_feederMotor.stopMotor();
    }

    public double getFlywheelPower(double distance) {
        return shooterFlywheelPower.get(distance);
    }

}
