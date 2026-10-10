package frc.robot.commandGroups.ShootCommandGroups;

import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;
import org.wpilib.command2.Commands;
import org.wpilib.command2.ParallelCommandGroup;

public class ShootBasic extends ParallelCommandGroup {
  public ShootBasic(
      DoubleSupplier speed,
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      HopperSubsystem hopperSubsystem) {
    addCommands(
        shooterSubsystem.shootAtSpeedCommand(speed),
        Commands.waitUntil(shooterSubsystem::isAtSpeed)
            .andThen(hopperSubsystem.runHopperUntilInterruptedCommand()));
  }
}
