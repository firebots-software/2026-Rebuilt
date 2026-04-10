package frc.robot.commands.DebugCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PeterSubsystem;

public class LeftShooterTest extends Command {
  private PeterSubsystem peterSubsystem;

  /**
   * Test the left flywheels on the shooter
   *
   * @param peterSubsystem the subsystem this command uses
   */
  public LeftShooterTest(PeterSubsystem peterSubsystem) {
    this.peterSubsystem = peterSubsystem;
    addRequirements(peterSubsystem);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    peterSubsystem.spinLeftShooter();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    peterSubsystem.stopLeftShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // wait for x seconds
  }
}
