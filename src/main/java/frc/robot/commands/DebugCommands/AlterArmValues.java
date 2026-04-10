package frc.robot.commands.DebugCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class AlterArmValues extends Command {
  private double increaseBy;

  public AlterArmValues(double increaseBy) {
    this.increaseBy = increaseBy;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.Arm.ARM_INTERMAP_OFFSET += increaseBy;
    Constants.Arm.UPDATE_INTERMAP();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
