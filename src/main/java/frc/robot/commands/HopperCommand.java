package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.subsystems.HopperSubsystem;

public class HopperCommand extends Command {
    private HopperSubsystem m_hopperSubsystem;
    private final double kFlywheelSpeed;
    private final double kFeederSpeed;
    private final double kDelay;

    public HopperCommand(HopperSubsystem hopperSubsystem, double flywheelSpeed, double feederSpeed, double delay) {
        m_hopperSubsystem = hopperSubsystem;
        kFlywheelSpeed = flywheelSpeed;
        kFeederSpeed = feederSpeed;
        kDelay = delay;
        addRequirements(hopperSubsystem);
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(m_hopperSubsystem.setMotorSpeed(kFlywheelSpeed, kFeederSpeed, kDelay));
    }

    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance().schedule(m_hopperSubsystem.setMotorSpeed(0, 0, 0));
        m_hopperSubsystem.stopMotors();
    }
}