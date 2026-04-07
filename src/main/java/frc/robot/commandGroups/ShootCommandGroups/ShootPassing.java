package frc.robot.commandGroups.ShootCommandGroups;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ShootPassing extends ParallelCommandGroup {
  public ShootPassing(
      DoubleSupplier translationalX,
      DoubleSupplier translationalY,
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      HopperSubsystem hopper,
      CommandSwerveDrivetrain drivetrain,
      BooleanSupplier redside) {
    Translation2d passingTranslation = drivetrain.getPassingTarget(redside);
    DoubleSupplier dist =
        () -> drivetrain.getPose().getTranslation().getDistance(passingTranslation);

    addCommands(
        shooter.shootAtSpeedHoodCommand(
            () -> shooter.grabTargetShootingSpeed(dist.getAsDouble()),
            () -> shooter.grabTargetHoodAngle(dist.getAsDouble())),
        new SwerveJoystickCommand(
            translationalX,
            translationalY,
            () -> 0.0,
            () -> 1.0,
            () -> false,
            () -> false,
            () -> true,
            redside,
            drivetrain),
        Commands.waitUntil(shooter::isShooterReady)
            .andThen(
                Commands.parallel(
                    hopper.runHopperUntilInterruptedCommand(
                        () -> Constants.Hopper.TARGET_SURFACE_SPEED_MPS, () -> true),
                    intake.powerRetractRollersCommand())));
  }
}
