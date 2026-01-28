package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAtSpeed extends Command {
  private final ShooterSubsystem shooterSubsystem;

  public ShootAtSpeed(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooterSubsystem.setSpeed(Constants.Shooter.SHOOT_FOR_AUTO);
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
