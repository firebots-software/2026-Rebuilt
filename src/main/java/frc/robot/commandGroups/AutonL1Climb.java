package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutonL1Climb extends SequentialCommandGroup {
  public AutonL1Climb(ClimberSubsystem climberSubsystem, CommandSwerveDrivetrain swerveDrivetrain) {
    addCommands(
        climberSubsystem.PullUpCommand(Constants.Climber.PullUp.L1_REACH_POS),
        climberSubsystem.SitUpCommand(Constants.Climber.SitUp.SIT_UP_ANGLE),
        new DriveToPose(swerveDrivetrain),
        climberSubsystem.PullUpCommand(Constants.Climber.PullUp.PULL_DOWN_POS));
  }
}
