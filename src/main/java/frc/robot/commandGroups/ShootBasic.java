package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ShootBasic extends ParallelCommandGroup {
  public ShootBasic(
      DoubleSupplier speed,
      BooleanSupplier readyToShoot,
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      HopperSubsystem hopperSubsystem) {
    addCommands(
        shooterSubsystem.shootAtSpeedCommand(speed),
        hopperSubsystem
            .runHopperUntilInterruptedCommand(Constants.Hopper.TARGET_SURFACE_SPEED_MPS, readyToShoot),
        intakeSubsystem.runRollersUntilInterruptedCommand(
            Constants.Intake.Rollers.TARGET_ROLLER_RPS));
  }
}