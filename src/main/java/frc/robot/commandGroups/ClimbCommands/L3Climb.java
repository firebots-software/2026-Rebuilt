package frc.robot.commandGroups.ClimbCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class L3Climb extends SequentialCommandGroup {
  public L3Climb(
      ClimberSubsystem climberSubsystem,
      CommandSwerveDrivetrain swerveDrivetrain,
      Pose2d poseToDriveTo) {
    addCommands(
        new L1Climb(climberSubsystem, swerveDrivetrain, poseToDriveTo),
        climberSubsystem.L3ClimbCommand(),
        climberSubsystem.brakeCommand());
  }
}
