package frc.robot.commandGroups.ShootCommandGroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Targeting;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ShootWithAim extends ParallelCommandGroup {
  public ShootWithAim(
      DoubleSupplier translationalX,
      DoubleSupplier translationalY,
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      HopperSubsystem hopperSubsystem,
      CommandSwerveDrivetrain drivetrain,
      BooleanSupplier redside,
      BooleanSupplier manualOverride) {
    DoubleSupplier dist =
        () ->
            (drivetrain
                .getPose()
                .getTranslation()
                .getDistance(drivetrain.getVirtualTarget(redside, () -> false)));
    addCommands(
        Commands.either(
            Commands.parallel( // shoot without aim
                shooterSubsystem.shootAtSpeedHoodCommand(
                    44.2, Constants.Shooter.Hood.MAX_HOOD_POSITION),
                Commands.waitUntil(shooterSubsystem::isShooterReady)
                    .andThen(
                        Commands.parallel(
                            hopperSubsystem.runHopperUntilInterruptedCommand(),
                            intakeSubsystem.powerRetractRollersCommand()))),
            Commands.parallel( // shoot with aim
                shooterSubsystem.shootAtSpeedHoodCommand(
                    () -> shooterSubsystem.grabTargetShootingSpeed(dist.getAsDouble()),
                    () -> shooterSubsystem.grabTargetHoodAngle(dist.getAsDouble())),
                new SwerveJoystickCommand(
                    translationalX,
                    translationalY,
                    () -> 0.0,
                    () -> 1.0,
                    () -> true,
                    () -> true,
                    () -> false,
                    redside,
                    drivetrain,
                    () -> true,
                    () -> true),
                Commands.waitUntil(shooterSubsystem::isAtSpeed)
                    .andThen(
                        Commands.parallel(
                            hopperSubsystem.runHopperUntilInterruptedCommand(
                                () -> Constants.Hopper.TARGET_SURFACE_SPEED_MPS,
                                () ->
                                    Targeting.pointingAtTarget(
                                        drivetrain
                                                .travelAngleTo(
                                                    new Pose2d(
                                                        drivetrain.getVirtualTarget(
                                                            redside, () -> false),
                                                        new Rotation2d()))
                                                .getRadians()
                                            + Math.PI,
                                        drivetrain)),
                            intakeSubsystem.powerRetractThenAgitateArmCommand()))),
            manualOverride));
  }
}
