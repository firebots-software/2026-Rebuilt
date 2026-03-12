package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

@Deprecated
public class ReverseIntakeAndHopper extends ParallelCommandGroup {
  public ReverseIntakeAndHopper(IntakeSubsystem intakeSubsystem, HopperSubsystem hopperSubsystem) {
    addCommands(
        intakeSubsystem.runRollersUntilInterruptedCommand(
            -Constants.Intake.Rollers.TARGET_ROLLER_RPS),
        hopperSubsystem.runHopperUntilInterruptedCommand(
            -Constants.Hopper.TARGET_SURFACE_SPEED_MPS));
  }
}
