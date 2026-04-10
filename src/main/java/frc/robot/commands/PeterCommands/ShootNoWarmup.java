package frc.robot.commands.PeterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PeterSubsystem;

public class ShootNoWarmup extends Command {
  private PeterSubsystem peterSubsystem;
  private boolean auton;

  /**
   * Creates a new ExampleCommand.
   *
   * @param peterSubsystem The subsystem used by this command.
   */
  public ShootNoWarmup(PeterSubsystem peterSubsystem, boolean auton) {
    this.peterSubsystem = peterSubsystem;
    this.auton = auton;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(peterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    peterSubsystem.spinLeftShooter();
    peterSubsystem.spinRightShooter();
    peterSubsystem.spinUpPreShooterVoltage();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    peterSubsystem.stopShooter(auton);
    peterSubsystem.stopPreShooterMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
