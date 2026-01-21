package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCommands.BackupShooter;
import frc.robot.commands.IntakeCommands.RunIntakeUntilDetection;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Intake extends SequentialCommandGroup {
  public Intake(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    addCommands(
        new RunIntakeUntilDetection(intakeSubsystem, shooterSubsystem),
        new BackupShooter(shooterSubsystem, intakeSubsystem));
  }
}
