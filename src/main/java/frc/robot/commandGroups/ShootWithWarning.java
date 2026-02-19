package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
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

    addCommands(
        new SwerveJoystickCommandWithPointing(
            frontBackFunction,
            leftRightFunction,
            () -> 1f,
            () -> true,
            () -> Targeting.targetAngle(target, drivetrain),
            drivetrain),
        Commands.runEnd(
            () ->
                joystick.setRumble(
                    RumbleType.kBothRumble,
                    (Targeting.amtToRumble(drivetrain, target).getAsDouble())),
            () -> joystick.setRumble(RumbleType.kBothRumble, (0d))),
        new ShootBasic(
            () ->
                Units.metersToFeet(
                    Targeting.shootingSpeed(
                        target, drivetrain, Constants.Shooter.TARGETING_CALCULATION_PRECISION)),
            () -> (Targeting.pointingAtTarget(target, drivetrain) && shooter.isAtSpeed()),
            shooter,
            intake,
            hopper));
  }
}
