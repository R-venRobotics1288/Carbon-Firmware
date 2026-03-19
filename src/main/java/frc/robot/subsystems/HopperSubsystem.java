package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Configs.HopperConfigs;
import frc.robot.Constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase {

    private final SparkFlex m_feederMotor;
    private final SparkFlex m_flywheelSpark;
    private final SparkFlex m_leftFlywheelMotor;
    private final SparkFlex m_agitatorMotor;
    private final RelativeEncoder m_agitatorEncoder;
    private final RelativeEncoder m_flywheelEncoder;
    private final InterpolatingDoubleTreeMap m_shooterFlywheelPower;
    private PIDController agitatorPID;

    private final SparkClosedLoopController m_flywheelClosedLoopController;

    public HopperSubsystem() {
        m_feederMotor = new SparkFlex(HopperConstants.kFeederCANID, MotorType.kBrushless);


        m_flywheelSpark = new SparkFlex(HopperConstants.kRightFlywheelCANID, MotorType.kBrushless);
        m_flywheelEncoder = m_flywheelSpark.getEncoder();
        m_flywheelEncoder.setPosition(0);
        m_flywheelClosedLoopController = m_flywheelSpark.getClosedLoopController();
         m_flywheelSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        m_leftFlywheelMotor = new SparkFlex(HopperConstants.kLeftFlywheelCANID, MotorType.kBrushless);
        m_agitatorMotor = new SparkFlex(HopperConstants.kAgitatorCANID, MotorType.kBrushless);
        m_agitatorEncoder = m_agitatorMotor.getEncoder();

        m_feederMotor.configure(HopperConfigs.intakeConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_flywheelSpark.configure(HopperConfigs.shooterConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_leftFlywheelMotor.configure(HopperConfigs.shooterConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_agitatorMotor.configure(HopperConfigs.intakeConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_shooterFlywheelPower = new InterpolatingDoubleTreeMap(); //In Meters
        m_shooterFlywheelPower.put(1.0, 0.6); 
        m_shooterFlywheelPower.put(2.0, 0.7);
        m_shooterFlywheelPower.put(3.0, 0.8);

        agitatorPID = new PIDController(0.0025, 0.006, 0.00015);

        setDefaultCommand(
                runOnce(
                        () -> {
                            m_flywheelSpark.disable();
                            m_leftFlywheelMotor.disable();
                            m_feederMotor.disable();
                            m_agitatorMotor.disable();
                        })
                        .andThen(run(() -> {
                        }))
                        .withName("Idle"));

    }

    public Command shootCommand(double flywheelSpeed, double feederMotorSpeed, double delay, boolean agitator) {
        return Commands.sequence(
                runOnce(() -> {
                    if (agitator) {
                        agitatorPID.setSetpoint(HopperConstants.kAgitatorMotorSpeed);
                    } else {
                        agitatorPID.setSetpoint(0);
                    }
                }),
                Commands.parallel(

                        // Run the shooter flywheel at the desired setpoint using feedforward and
                        // feedback
                        run(
                                () -> {
                                    m_flywheelSpark.set(flywheelSpeed);
                                    m_leftFlywheelMotor.set(-flywheelSpeed);
                                    double val = agitatorPID.calculate(m_agitatorEncoder.getVelocity());
                                    System.out.print(val);
                                    System.out.print(" : ");
                                    System.out.println(m_agitatorEncoder.getVelocity());
                                    m_agitatorMotor.setVoltage(MathUtil.clamp(val, 0.0, 12));
                                }),

                        // Wait until the shooter has reached the setpoint, and then run the feeder
                        Commands.waitSeconds(delay).andThen(() -> {
                            m_feederMotor.set(feederMotorSpeed);
                        })))
                .withName("Shoot");
    }

    public void stopMotors() {
        m_flywheelSpark.set(0);
        m_feederMotor.set(0);
        m_flywheelSpark.stopMotor();
        m_feederMotor.stopMotor();
    }

    public double getFlywheelPower(double distance) {
        return m_shooterFlywheelPower.get(distance);
    }

    public void setMotorSpeed(double power, double kShooterFeederMotorSpeed) {
        m_flywheelClosedLoopController.setSetpoint(kShooterFeederMotorSpeed, null);
        m_feederMotor.set(kShooterFeederMotorSpeed);
    }

}
