package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class ExtendIntake extends ParallelCommandGroup {
  public ExtendIntake(IntakeSubsystem intake) {
    addCommands(
        intake.intakeUntilInterruptedCommand());
  }
}
