package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ExpelToFullyRetractWithTime extends SequentialCommandGroup {
  public ExpelToFullyRetractWithTime(
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      HopperSubsystem hopperSubsystem,
      double timeSeconds) {
    addCommands(
        shooterSubsystem.shootAtSpeedCommand().withTimeout(timeSeconds),
        intakeSubsystem.armToDegrees(Constants.Intake.Arm.ARM_POS_RETRACTED));
  }
}
