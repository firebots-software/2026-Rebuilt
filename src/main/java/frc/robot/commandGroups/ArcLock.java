package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.Landmarks;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommandInArc;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Targeting;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ArcLock extends ParallelCommandGroup {
  public ArcLock(
      CommandSwerveDrivetrain drivetrain,
      ShooterSubsystem shooter,
      DoubleSupplier tangentialVelocitySupplier,
      Pose2d target,
      BooleanSupplier redside,
      CommandXboxController joystick) {

    addCommands(
        new SwerveJoystickCommandInArc(
            target,
            tangentialVelocitySupplier,
            () -> 1f,
            () -> false,
            () -> Targeting.targetAngle(target, drivetrain),
            drivetrain,
            redside),
        Commands.runEnd(
            () ->
                joystick.setRumble(
                    RumbleType.kBothRumble,
                    (Targeting.amtToRumble(drivetrain, target).getAsDouble())),
            () -> joystick.setRumble(RumbleType.kBothRumble, (0d))),
        shooter.shootAtSpeedCommand(
            () ->
                Targeting.shootingSpeed(
                    target, drivetrain, Constants.Shooter.TARGETING_CALCULATION_PRECISION)));
  }

  public ArcLock(
      CommandSwerveDrivetrain drivetrain,
      ShooterSubsystem shooter,
      DoubleSupplier tangentialVelocitySupplier,
      BooleanSupplier redside,
      CommandXboxController joystick) {
    this(
        drivetrain,
        shooter,
        tangentialVelocitySupplier,
        redside.getAsBoolean() ? Landmarks.RED_HUB : Landmarks.BLUE_HUB,
        redside,
        joystick);
  }
}
