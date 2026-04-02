package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.Landmarks;
import frc.robot.MathUtils.Vector3;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommandInArc;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Targeting;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ArcAroundAndShoot extends ParallelCommandGroup {
  public ArcAroundAndShoot(
      CommandSwerveDrivetrain drivetrain,
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      HopperSubsystem hopper,
      DoubleSupplier tangentialVelocitySupplier,
      Pose3d target,
      BooleanSupplier redside,
      CommandXboxController joystick) {

    addCommands(
        new SwerveJoystickCommandInArc(
            target,
            tangentialVelocitySupplier,
            () -> 1f,
            (BooleanSupplier) () -> false,
            (Supplier<Translation2d>)
                () ->
                    Vector3.toTranslation2d(
                        Targeting.positionToTarget(
                            target, drivetrain, Constants.Shooter.TARGETING_CALCULATION_PRECISION)),
            drivetrain),
        Commands.runEnd(
            () ->
                joystick.setRumble(
                    RumbleType.kBothRumble,
                    (Targeting.amtToRumble(drivetrain, target).getAsDouble())),
            () -> joystick.setRumble(RumbleType.kBothRumble, (0d))),
        new ShootBasic(
            () ->
                Targeting.shootingSpeed(
                    target, drivetrain, Constants.Shooter.TARGETING_CALCULATION_PRECISION),
            () -> (Targeting.pointingAtTarget(target, drivetrain) && shooter.isAtSpeed()),
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
      BooleanSupplier redside,
      CommandXboxController joystick) {
    this(
        drivetrain,
        shooter,
        intake,
        hopper,
        tangentialVelocitySupplier,
        redside.getAsBoolean() ? Landmarks.RED_HUB : Landmarks.BLUE_HUB,
        redside,
        joystick);
  }
}
