package frc.robot.commandGroups.ClimbCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class L3Climb extends SequentialCommandGroup {
  public L3Climb(ClimberSubsystem climberSubsystem, CommandSwerveDrivetrain swerveDrivetrain) {
    addCommands(
        new L1Climb(climberSubsystem, swerveDrivetrain),
        climberSubsystem.L3ClimbCommand(),
        climberSubsystem.brakeCommand());
  }
}
