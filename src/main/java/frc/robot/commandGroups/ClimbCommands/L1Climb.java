package frc.robot.commandGroups.ClimbCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class L1Climb extends SequentialCommandGroup {
  public L1Climb(
      ClimberSubsystem climberSubsystem,
      CommandSwerveDrivetrain swerveDrivetrain,
      Pose2d poseToDriveTo) {
    addCommands(
        climberSubsystem.sitUpCommand(Constants.Climber.SitUp.SIT_UP_ANGLE_DEGREES),
        climberSubsystem.pullUpCommand(Constants.Climber.PullUp.L1_REACH_POS),
        new DriveToPose(swerveDrivetrain, () -> poseToDriveTo),
        climberSubsystem.pullUpCommand(Constants.Climber.PullUp.PULL_DOWN_POS));
  }
}
