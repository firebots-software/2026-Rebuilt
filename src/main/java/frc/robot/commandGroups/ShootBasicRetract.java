package frc.robot.commandGroups;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
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
        Commands.waitUntil(shooterSubsystem::isAtSpeed)
            .andThen(
                new ParallelCommandGroup(
                    hopperSubsystem.runHopperUntilInterruptedCommand(),
                    intakeSubsystem
                        .powerRetractRollersCommand()
                        .beforeStarting(
                            Commands.waitSeconds(Constants.Intake.Arm.POWER_RETRACT_DELAY)))));
  }

  public ShootBasicRetract(
      DoubleSubscriber speed,
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      HopperSubsystem hopperSubsystem) {
    addCommands(
        shooterSubsystem.shootAtSpeedCommand(speed),
        Commands.waitUntil(shooterSubsystem::isAtSpeed)
            .andThen(
                new ParallelCommandGroup(
                    hopperSubsystem.runHopperUntilInterruptedCommand(),
                    intakeSubsystem
                        .powerRetractRollersCommand()
                        .beforeStarting(
                            Commands.waitSeconds(Constants.Intake.Arm.POWER_RETRACT_DELAY)))));
  }
}
