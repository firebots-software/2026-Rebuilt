package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class SetMuscleUpToAngle extends Command {
  ClimberSubsystem climber;
  double angle;

  public SetMuscleUpToAngle(ClimberSubsystem climber, double angle) {
    this.climber = climber;
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setMuscleUpPosition(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.isMuscleUpAtPosition();
  }
}
