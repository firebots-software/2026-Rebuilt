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
      DoubleSupplier angle,
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      HopperSubsystem hopperSubsystem) {
    addCommands(
        shooterSubsystem.shootAtSpeedHoodCommand(speed, angle),
        Commands.waitUntil(shooterSubsystem::isAtSpeed)
            .andThen(
                Commands.parallel(
                    hopperSubsystem.runHopperUntilInterruptedCommand(),
                    intakeSubsystem.powerRetractRollersCommand())));
  }
}
