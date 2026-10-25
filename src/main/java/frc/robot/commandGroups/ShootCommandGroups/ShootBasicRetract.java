package frc.robot.commandGroups.ShootCommandGroups;

import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;
import org.wpilib.command2.Commands;
import org.wpilib.command2.ParallelCommandGroup;

public class ShootBasicRetract extends ParallelCommandGroup {
  public ShootBasicRetract(
      DoubleSupplier speed,
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      HopperSubsystem hopperSubsystem) {
    addCommands(
        shooterSubsystem.shootAtSpeedCommand(speed),
        Commands.waitUntil(shooterSubsystem::isAtSpeed)
            .andThen(
                Commands.parallel(
                    hopperSubsystem.runHopperUntilInterruptedCommand(),
                    intakeSubsystem.powerRetractRollersCommand())));
  }
}
