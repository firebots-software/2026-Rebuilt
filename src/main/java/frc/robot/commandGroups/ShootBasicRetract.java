package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ShootBasicRetract extends ParallelCommandGroup {
  public ShootBasicRetract(
      DoubleSupplier speed,
      BooleanSupplier readyToShoot,
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      HopperSubsystem hopperSubsystem) {
    addCommands(
        (shooterSubsystem.shootAtSpeedCommand(speed)),
        Commands.waitUntil(() -> (shooterSubsystem.isAtSpeed() && readyToShoot.getAsBoolean()))
            .andThen(
                hopperSubsystem
                    .runHopperUntilInterruptedCommand()
                    .alongWith(
                        intakeSubsystem
                            .setArmToDegreesCommand(Constants.Intake.Arm.ARM_POS_RETRACTED)
                            .beforeStarting(Commands.waitSeconds(0.5)))));

    // addCommands(
    //     shooterSubsystem.shootAtSpeedCommand(speed),
    //     hopperSubsystem.runHopperUntilInterruptedCommand(
    //         Constants.Hopper.TARGET_SURFACE_SPEED_MPS, readyToShoot),
    //     intakeSubsystem.runRollersUntilInterruptedCommand(
    //         Constants.Intake.Rollers.TARGET_ROLLER_RPS));
  }
}
