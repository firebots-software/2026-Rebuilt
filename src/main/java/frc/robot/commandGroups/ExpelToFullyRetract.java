package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ExpelToFullyRetract extends SequentialCommandGroup {
  public ExpelToFullyRetract(
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      HopperSubsystem hopperSubsystem) {
    addCommands(
        shooterSubsystem.shootAtSpeedCommand().until(hopperSubsystem::isHopperSufficientlyEmpty),
        intakeSubsystem.armToDegrees(Constants.Intake.Arm.ARM_POS_RETRACTED));
  }
}
