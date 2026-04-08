package frc.robot.commandGroups.ShootCommandGroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommandWithPointing;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Targeting;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ShootPassing extends ParallelCommandGroup {
  public ShootPassing(
      DoubleSupplier translationalX,
      DoubleSupplier translationalY,
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      HopperSubsystem hopper,
      CommandSwerveDrivetrain drivetrain,
      BooleanSupplier redside) {
    Supplier<Translation2d> passingTranslation =
        () ->
            Targeting.computeVirtualTarget(
                new Pose2d(drivetrain.getPassingTarget(redside), new Rotation2d()), drivetrain);
    DoubleSupplier dist =
        () -> drivetrain.getPose().getTranslation().getDistance(passingTranslation.get());

    addCommands(
        shooter.shootAtSpeedHoodCommand(
            () -> shooter.grabTargetShootingSpeed(dist.getAsDouble()),
            () -> Constants.Shooter.Hood.MAX_HOOD_POSITION),
        new SwerveJoystickCommandWithPointing(
            translationalX, translationalY, () -> 0.0, () -> false, passingTranslation, drivetrain),
        Commands.waitUntil(
                () ->
                    (shooter.isShooterReady()
                        && Targeting.pointingAtTarget(
                            new Pose2d(drivetrain.getPassingTarget(redside), new Rotation2d()),
                            drivetrain)))
            .andThen(
                Commands.parallel(
                    hopper.runHopperUntilInterruptedCommand(
                        () -> Constants.Hopper.TARGET_SURFACE_SPEED_MPS, () -> true),
                    intake.powerRetractRollersCommand())));
  }
}
