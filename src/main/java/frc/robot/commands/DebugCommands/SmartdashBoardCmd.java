package frc.robot.commands.DebugCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SmartdashBoardCmd extends Command {
  private String k, v;

  public SmartdashBoardCmd(String key, String value) {
    this.v = value;
    this.k = key;
  }

  @Override
  public void initialize() {
    SmartDashboard.putString(k, v);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {}
}
