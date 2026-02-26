package frc.robot.commandGroups.ClimbCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class L1ClimbAuto extends SequentialCommandGroup {
  public L1ClimbAuto(
      ClimberSubsystem climberSubsystem,
      CommandSwerveDrivetrain swerveDrivetrain,
      Pose2d poseToDriveTo) {
    addCommands(
        climberSubsystem.SitUpCommand(Constants.Climber.SitUp.SIT_UP_ANGLE_DEGREES),
        climberSubsystem.PullUpCommand(Constants.Climber.PullUp.L1_REACH_POS),
        new DriveToPose(swerveDrivetrain, () -> poseToDriveTo),
        climberSubsystem.PullUpCommand(Constants.Climber.PullUp.PULL_DOWN_POS_L1_AUTO));
  }
}
