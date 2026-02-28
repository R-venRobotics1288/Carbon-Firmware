package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.HopperSubsystem;

public class IntakeCommand extends Command {
    private HopperSubsystem m_hopperSubsystem;

    public IntakeCommand(HopperSubsystem hopperSubsystem) {
        m_hopperSubsystem = hopperSubsystem;
        addRequirements(hopperSubsystem);
    }

    @Override
    public void initialize() {
        m_hopperSubsystem.setIntakeMotorSpeed(HopperConstants.kIntakeMotorSpeed, -HopperConstants.kShooterMotorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_hopperSubsystem.stopIntake();
    }
}