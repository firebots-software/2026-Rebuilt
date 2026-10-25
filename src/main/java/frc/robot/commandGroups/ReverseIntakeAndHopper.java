package frc.robot.commandGroups;

import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import org.wpilib.command2.ParallelCommandGroup;

public class ReverseIntakeAndHopper extends ParallelCommandGroup {
  public ReverseIntakeAndHopper(IntakeSubsystem intakeSubsystem, HopperSubsystem hopperSubsystem) {
    addCommands(
        intakeSubsystem.runRollersUntilInterruptedCommand(
            -Constants.Intake.Rollers.TARGET_ROLLER_RPS),
        hopperSubsystem.runHopperUntilInterruptedCommand(
            -Constants.Hopper.TARGET_SURFACE_SPEED_MPS));
  }
}
