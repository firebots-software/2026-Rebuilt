package frc.robot.commandGroups.ClimbCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.BooleanSupplier;

public class L2Climb extends SequentialCommandGroup {
  public L2Climb(
      ClimberSubsystem climberSubsystem,
      CommandSwerveDrivetrain swerveDrivetrain,
      BooleanSupplier redside) {
    addCommands(
        new L1Climb(climberSubsystem, swerveDrivetrain, redside),
        climberSubsystem.L2ClimbCommand(),
        climberSubsystem.brakeCommand());
  }
}
