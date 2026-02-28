package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.HopperSubsystem;

public class ShootCommand extends Command {
    private final HopperSubsystem m_HopperSubsystem;

    public ShootCommand(HopperSubsystem hopperSubsystem) {
        m_HopperSubsystem = hopperSubsystem;
        addRequirements(m_HopperSubsystem);
    }

    @Override
    public void initialize() {
        m_HopperSubsystem.setShooterMotorSpeed(HopperConstants.kShooterMotorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_HopperSubsystem.stopShooter();
    }
}
