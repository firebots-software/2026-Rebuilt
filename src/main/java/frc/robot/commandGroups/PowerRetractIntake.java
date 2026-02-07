package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class PowerRetractIntake extends ParallelCommandGroup {
  public PowerRetractIntake(IntakeSubsystem intakeSubsystem, HopperSubsystem hopperSubsystem) {
    addCommands(
        intakeSubsystem
            .armToDegrees(Constants.Intake.Arm.ARM_POS_RETRACTED)
            .onlyIf(() -> hopperSubsystem.isHopperSufficientlyEmpty()));
  }
}
