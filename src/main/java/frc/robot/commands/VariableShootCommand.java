package frc.robot.commands;

import static frc.robot.Constants.HopperConstants.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;

public class VariableShootCommand extends Command {
  private final Translation2d HUB_POSITION;

  private final HopperSubsystem hopperSubsystem;
  private final DriveSubsystem driveSubsystem;

  private double distance;
  private double power;

  public VariableShootCommand(HopperSubsystem hopperSubsystem, DriveSubsystem driveSubsystem) {
    this.hopperSubsystem = hopperSubsystem;
    this.driveSubsystem = driveSubsystem;
    HUB_POSITION = DriverStation.getAlliance().get() == Alliance.Red ? RED_HUB_POSITION : BLUE_HUB_POSITION;
    addRequirements(hopperSubsystem);
  }

  @Override
  public void execute() {
    distance = driveSubsystem.getEstimatedRobotPose().getTranslation().getDistance(HUB_POSITION);
    power = hopperSubsystem.getPower(distance);
    hopperSubsystem.setMotorSpeeds(-INTAKE_SPEED, power);
  }

  @Override
  public void end(boolean interrupted) {
    hopperSubsystem.stopIntake();
    hopperSubsystem.stopShooter();
  }
}
