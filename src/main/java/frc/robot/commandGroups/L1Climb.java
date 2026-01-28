package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ClimberCommands.SetPullUpToPosition;
import frc.robot.commands.ClimberCommands.SetSitUpToAngle;
import frc.robot.subsystems.ClimberSubsystem;

public class L1Climb extends SequentialCommandGroup {
  public L1Climb(ClimberSubsystem climber) {
    addCommands(
        new SetPullUpToPosition(climber, Constants.Climber.PullUp.REACH_POS),
        new SetSitUpToAngle(climber, Constants.Climber.SitUp.SIT_UP_ANGLE),
        new SetPullUpToPosition(climber, Constants.Climber.PullUp.PULL_DOWN_POS));
  }
}
