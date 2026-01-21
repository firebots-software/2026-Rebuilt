package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class BackupShooter extends Command {
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;

  public BackupShooter(ShooterSubsystem shooter, IntakeSubsystem intake) {
    shooterSubsystem = shooter;
    intakeSubsystem = intake;
    addRequirements(shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.runPreShooterAtRPS(-40);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopPreShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !intakeSubsystem.beamBroken();
  }
}
