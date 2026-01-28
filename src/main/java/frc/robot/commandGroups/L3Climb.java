package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ClimberCommands.SetMuscleUpToAngle;
import frc.robot.commands.ClimberCommands.SetPullUpToPosition;
import frc.robot.commands.ClimberCommands.SetSitUpToAngle;
import frc.robot.subsystems.ClimberSubsystem;

public class L3Climb extends SequentialCommandGroup {
  public L3Climb(ClimberSubsystem climber) {
    for (int i = 0; i < 3; i++) {
      addCommands(
          new SetPullUpToPosition(climber, Constants.Climber.PullUp.REACH_POS),
          new SetSitUpToAngle(climber, Constants.Climber.SitUp.SIT_UP_ANGLE),
          new SetMuscleUpToAngle(climber, Constants.Climber.MuscleUp.MUSCLE_UP_BACK),
          new SetPullUpToPosition(climber, Constants.Climber.PullUp.PULL_DOWN_POS),
          new SetMuscleUpToAngle(climber, Constants.Climber.MuscleUp.MUSCLE_UP_FORWARD),
          new SetSitUpToAngle(climber, Constants.Climber.SitUp.SIT_BACK_ANGLE));
    }
  }
}
