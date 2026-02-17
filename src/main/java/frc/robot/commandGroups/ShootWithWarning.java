package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.MathUtils.Vector3;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommandWithPointing;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Targeting;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ShootWithWarning extends ParallelCommandGroup {
  public ShootWithWarning(
      CommandSwerveDrivetrain drivetrain,
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      HopperSubsystem hopper,
      DoubleSupplier tangentialVel,
      Pose3d target,
      BooleanSupplier redside,
      Joystick joystick,
      DoubleSupplier frontBackFunction,
      DoubleSupplier leftRightFunction) {

    double distMeters =
        Vector3.subtract(new Vector3(drivetrain.getCurrentState().Pose), new Vector3(target))
            .magnitude();
    addCommands(
        new SwerveJoystickCommandWithPointing(
            frontBackFunction,
            leftRightFunction,
            () -> 1f,
            () -> true,
            () -> Targeting.targetAngle(target, drivetrain),
            drivetrain),
        new InstantCommand(
            () ->
                joystick.setRumble(
                    RumbleType.kBothRumble,
                    (Units.metersToFeet(distMeters) > Constants.Shooter.MAX_DIST_FT
                            || Units.metersToFeet(distMeters) < Constants.Shooter.MIN_DIST_FT)
                        ? .5d
                        : 0d)),
        new Shoot(
            Units.metersToFeet(
                Targeting.shootingSpeed(
                    target, drivetrain, Constants.Shooter.TARGETING_CALCULATION_PRECISION)),
            () -> Targeting.pointingAtTarget(target, drivetrain),
            shooter,
            intake,
            hopper));
  }
}
