package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends ParallelCommandGroup {
  public Shoot(
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      HopperSubsystem hopperSubsystem) {
    addCommands(
        new WarmUpAndShoot(shooterSubsystem, hopperSubsystem),
        intakeSubsystem.armToDegrees(Constants.Intake.Arm.ARM_POS_RETRACTED));
  }
}
