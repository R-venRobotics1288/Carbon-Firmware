package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;

public class HopperCommand extends Command {
    private HopperSubsystem m_hopperSubsystem;
    private final double kFlywheelSpeed;
    private final double kFeederSpeed;

    public HopperCommand(HopperSubsystem hopperSubsystem, double flywheelSpeed, double feederSpeed) {
        m_hopperSubsystem = hopperSubsystem;
        kFlywheelSpeed = flywheelSpeed;
        kFeederSpeed = feederSpeed;
        addRequirements(hopperSubsystem);
    }

    @Override
    public void initialize() {
        m_hopperSubsystem.setMotorSpeed(kFlywheelSpeed, kFeederSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_hopperSubsystem.stopMotors();
    }
}