package frc.robot.commandGroups.ClimbCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class L1Climb extends SequentialCommandGroup {
  public L1Climb(ClimberSubsystem climberSubsystem, CommandSwerveDrivetrain swerveDrivetrain) {
    addCommands(
        climberSubsystem.PullUpCommand(Constants.Climber.PullUp.L1_REACH_POS),
        climberSubsystem.SitUpCommand(Constants.Climber.SitUp.SIT_UP_ANGLE),
        climberSubsystem.PullUpCommand(Constants.Climber.PullUp.PULL_DOWN_POS));
  }
}
