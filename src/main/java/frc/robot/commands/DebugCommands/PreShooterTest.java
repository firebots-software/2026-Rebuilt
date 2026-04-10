package frc.robot.commands.DebugCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PeterSubsystem;

public class PreShooterTest extends Command {
  private PeterSubsystem preShooter;

  public PreShooterTest(PeterSubsystem preShooter) {
    this.preShooter = preShooter;
    addRequirements(preShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SmartDashboard.getBoolean(this.toString(), true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    preShooter.spinUpPreShooterVoltage();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    preShooter.stopPreShooterMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
