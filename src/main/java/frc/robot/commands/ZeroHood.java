package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ZeroHood extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private double timesExceededCurrent;

  public ZeroHood(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.reduceHoodCurrentLimits();
    timesExceededCurrent = 0;
  }

  @Override
  public void execute() {
    shooterSubsystem.moveHoodWithVoltage();
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      shooterSubsystem.resetHoodPositionToZero();
    }
    shooterSubsystem.resetHoodCurrentLimits();
    shooterSubsystem.stopHood();
  }

  @Override
  public boolean isFinished() {
    if (shooterSubsystem.checkHoodCurrent()) {
      timesExceededCurrent++;
    } else {
      timesExceededCurrent = 0;
    }
    return timesExceededCurrent >= 10;
  }
}
