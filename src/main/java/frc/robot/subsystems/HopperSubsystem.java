package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.HopperConfigs;
import frc.robot.Constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase {

    private final SparkFlex m_shooterMotor;
    private final SparkFlex m_intakeMotor;

    public HopperSubsystem() {
        m_shooterMotor = new SparkFlex(HopperConstants.kShooterCANID, MotorType.kBrushless);
        m_intakeMotor = new SparkFlex(HopperConstants.kIntakeCANID, MotorType.kBrushless);

        m_shooterMotor.configure(HopperConfigs.shooterConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_intakeMotor.configure(HopperConfigs.intakeConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void setShooterMotorSpeed(double speed) {
        m_shooterMotor.set(speed);
    }

    public void stopShooter() {
        m_shooterMotor.stopMotor();
    }

    public void setIntakeMotorSpeed(double intakeSpeed, double shooterSpeed) {
        m_intakeMotor.set(intakeSpeed);
        m_shooterMotor.set(shooterSpeed);
    }

    public void stopIntake() {
        m_intakeMotor.stopMotor();
        m_shooterMotor.stopMotor();
    }
}
