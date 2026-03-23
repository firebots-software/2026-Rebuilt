package frc.robot.commandGroups.ShootCommandGroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class ShootBasicRetract extends ParallelCommandGroup {
  public ShootBasicRetract(
      DoubleSupplier speed,
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      HopperSubsystem hopperSubsystem) {
    addCommands(
        shooterSubsystem.shootAtSpeedCommand(speed),
        Commands.parallel(
                hopperSubsystem.runHopperUntilInterruptedCommand(),
                intakeSubsystem.powerRetractRollersCommand())
            .onlyIf(shooterSubsystem::isAtSpeed));
  }
}
