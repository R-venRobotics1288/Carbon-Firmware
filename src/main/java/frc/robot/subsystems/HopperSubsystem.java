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
    private final SparkFlex m_leftFlywheelSpark;
    private final SparkFlex m_rightFlywheelSpark;
    private final SparkFlex m_agitatorMotor;
    private final RelativeEncoder m_agitatorEncoder;
    private final RelativeEncoder m_leftFlywheelEncoder;
    private final RelativeEncoder m_rightFlywheelEncoder;
    private final InterpolatingDoubleTreeMap m_shooterFlywheelPower;
    private PIDController agitatorPID;

    private final SparkClosedLoopController m_rightFlywheelClosedLoopController;
    private final SparkClosedLoopController m_leftFlywheelClosedLoopController;

    public HopperSubsystem() {
        m_feederMotor = new SparkFlex(HopperConstants.kFeederCANID, MotorType.kBrushless);


        m_rightFlywheelSpark = new SparkFlex(HopperConstants.kRightFlywheelCANID, MotorType.kBrushless);
        m_rightFlywheelEncoder = m_rightFlywheelSpark.getEncoder();
        m_rightFlywheelEncoder.setPosition(0);
        m_rightFlywheelClosedLoopController = m_rightFlywheelSpark.getClosedLoopController();
        m_rightFlywheelSpark.configure(Configs.HopperConfigs.flywheelConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        m_leftFlywheelSpark = new SparkFlex(HopperConstants.kLeftFlywheelCANID, MotorType.kBrushless);
        m_leftFlywheelEncoder = m_leftFlywheelSpark.getEncoder();
        m_leftFlywheelEncoder.setPosition(0);
        m_leftFlywheelClosedLoopController = m_leftFlywheelSpark.getClosedLoopController();
        m_leftFlywheelSpark.configure(Configs.HopperConfigs.flywheelConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        m_agitatorMotor = new SparkFlex(HopperConstants.kAgitatorCANID, MotorType.kBrushless);
        m_agitatorEncoder = m_agitatorMotor.getEncoder();

        m_feederMotor.configure(HopperConfigs.intakeConfig, ResetMode.kResetSafeParameters,
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
                            m_leftFlywheelSpark.disable();
                            m_rightFlywheelSpark.disable();
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
                                    m_rightFlywheelSpark.set(flywheelSpeed);
                                    m_leftFlywheelSpark.set(flywheelSpeed);
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
        m_rightFlywheelSpark.set(0);
        m_leftFlywheelSpark.set(0);
        m_feederMotor.set(0);
        m_rightFlywheelSpark.stopMotor();
        m_leftFlywheelSpark.stopMotor();
        m_feederMotor.stopMotor();
    }

    public double getFlywheelPower(double distance) {
        return m_shooterFlywheelPower.get(distance);
    }

    public void setMotorSpeed(double power, double kShooterFeederMotorSpeed) {
        m_rightFlywheelClosedLoopController.setSetpoint(power, null);
        m_leftFlywheelClosedLoopController.setSetpoint(power, null);
        m_feederMotor.set(kShooterFeederMotorSpeed);
    }

}
