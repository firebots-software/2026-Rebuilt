package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Landmarks;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommandInArc;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Targeting;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ArcAroundAndShoot extends ParallelCommandGroup {
  public ArcAroundAndShoot(
      CommandSwerveDrivetrain drivetrain,
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      HopperSubsystem hopper,
      DoubleSupplier tangentialVelocitySupplier,
      Pose3d target,
      BooleanSupplier redside) {
    addCommands(
        new SwerveJoystickCommandInArc(
            target,
            tangentialVelocitySupplier,
            (DoubleSupplier) (() -> 1f),
            (BooleanSupplier) (() -> true),
            (DoubleSupplier) (() -> Targeting.targetAngle(target, drivetrain)),
            drivetrain,
            redside),
        new Shoot(
            Targeting.shootingSpeed(
                target, drivetrain, Constants.Shooter.TARGETING_CALCULATION_PRECISION),
            () -> Targeting.pointingAtTarget(target, drivetrain),
            shooter,
            intake,
            hopper));
  }

  public ArcAroundAndShoot(
      CommandSwerveDrivetrain drivetrain,
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      HopperSubsystem hopper,
      DoubleSupplier tangentialVelocitySupplier,
      BooleanSupplier redside) {
    this(
        drivetrain,
        shooter,
        intake,
        hopper,
        tangentialVelocitySupplier,
        redside.getAsBoolean() ? Landmarks.RED_HUB : Landmarks.BLUE_HUB,
        redside);
  }
}
