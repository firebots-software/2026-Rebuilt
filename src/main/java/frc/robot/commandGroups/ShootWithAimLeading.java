package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Landmarks;
import frc.robot.MathUtils.Vector2;
import frc.robot.MathUtils.Vector3;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommand;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommandInArc;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommandWithPointing;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.MiscUtils;
import frc.robot.util.Targeting;

import java.lang.annotation.Target;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ShootWithAimLeading extends ParallelCommandGroup {

  public ShootWithAimLeading(
      DoubleSupplier translationalX,
      DoubleSupplier translationalY,
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      HopperSubsystem hopperSubsystem,
      CommandSwerveDrivetrain drivetrain,
      BooleanSupplier redside,
      BooleanSupplier arcing,
      BooleanSupplier manualOverride) {
    
    Pose3d targetNoOffset = redside.getAsBoolean() ? Landmarks.RED_HUB : Landmarks.BLUE_HUB;
    Vector2 target =
        Vector3.toVector2(
            Targeting.positionToTarget(
                targetNoOffset,
                drivetrain,
                Constants.Shooter.TARGETING_CALCULATION_PRECISION));
    Vector2 curPose = Vector2.fromPose2d(drivetrain.getCurrentState().Pose);

    double dist = Vector2.dist(target, curPose);

    addCommands(
        Commands.either(
            Commands.parallel( // shoot without aim
                shooterSubsystem.shootAtSpeedHoodCommand(
                    44.2, Constants.Shooter.Hood.MAX_HOOD_POSITION),
                Commands.waitUntil(shooterSubsystem::isAtSpeed)
                    .andThen(
                        hopperSubsystem
                            .runHopperUntilInterruptedCommand()
                            .alongWith(
                                intakeSubsystem
                                    .powerRetractRollersCommand()
                                    .beforeStarting(
                                        Commands.waitSeconds(
                                            Constants.Intake.Arm.POWER_RETRACT_DELAY))))),
            Commands.parallel( // shoot with aim
                shooterSubsystem.shootAtSpeedHoodCommand(
                    () ->
                        shooterSubsystem.getTargetShootingSpeed(
                            dist),
                    () ->
                        shooterSubsystem.getTargetHoodAngle(
                            dist)),
                Commands.either(
                    new SwerveJoystickCommandInArc(
                        targetNoOffset,
                        translationalX,
                        translationalY,
                        (BooleanSupplier) () -> false,
                        (Supplier<Translation2d>) () -> Vector3.toTranslation2d(Targeting.positionToTarget(targetNoOffset, drivetrain, Constants.Shooter.TARGETING_CALCULATION_PRECISION)),
                        drivetrain
                    ),
                    new SwerveJoystickCommandWithPointing(
                        translationalX,
                        translationalY,
                        (DoubleSupplier) () -> 0.0,
                        (BooleanSupplier) () -> false,
                        (Supplier<Translation2d>) () -> Vector3.toTranslation2d(Targeting.positionToTarget(targetNoOffset, drivetrain, Constants.Shooter.TARGETING_CALCULATION_PRECISION)),
                        drivetrain
                    ),
                    arcing
                ),
                Commands.waitUntil(shooterSubsystem::isAtSpeed)
                    .andThen(
                        hopperSubsystem
                            .runHopperUntilInterruptedCommand(
                                () ->
                                    hopperSubsystem.grabHopperRecommendedSpeed(
                                        shooterSubsystem.getCurrentShooterWheelSpeedRPS()),
                                () ->
                                    (Targeting.pointingAtHub(redside, drivetrain)
                                        && (drivetrain.getSpeedMagnitude() <= 0.2)))
                            .alongWith(
                                intakeSubsystem
                                    .powerRetractRollersCommand()
                                    .beforeStarting(
                                        Commands.waitSeconds(
                                            Constants.Intake.Arm.POWER_RETRACT_DELAY))))),
            manualOverride));
  }
}
