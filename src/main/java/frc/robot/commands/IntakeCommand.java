package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.HopperSubsystem;

public class IntakeCommand extends Command {
    private HopperSubsystem m_hopper;
    
    public IntakeCommand(HopperSubsystem m_hopper) {
        this.m_hopper = m_hopper;
        addRequirements(m_hopper);
    }

    @Override
    public void initialize() { 
        m_hopper.setIntakeMotorSpeed(HopperConstants.kIntakeMotorSpeed, -HopperConstants.kShooterMotorSpeed);
        
    }

    @Override
    public void end(boolean interrupted) {
        m_hopper.setIntakeMotorSpeed(0, 0);
    }
}
