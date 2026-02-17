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
import java.util.function.DoubleSupplier;

public class LockOnCommand extends ParallelCommandGroup {
  public LockOnCommand(
      CommandSwerveDrivetrain drivetrain,
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      HopperSubsystem hopper,
      DoubleSupplier frontBackFunction,
      DoubleSupplier leftRightFunction,
      Pose3d target,
      Joystick joystick) {

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
        shooter.shootAtSpeedCommand(
            Units.metersToFeet(
                Targeting.shootingSpeed(
                    target, drivetrain, Constants.Shooter.TARGETING_CALCULATION_PRECISION))));
  }
}
