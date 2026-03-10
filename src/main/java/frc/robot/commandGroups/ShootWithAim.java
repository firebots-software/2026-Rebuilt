package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Landmarks;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.MiscUtils;
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
      BooleanSupplier redside) {

    addCommands(

        shooterSubsystem.shootAtSpeedCommand(
            () ->
                shooterSubsystem.grabTargetShootingSpeed(
                    MiscUtils.getDistanceToHub(redside, drivetrain))),

        new SwerveJoystickCommand(
            translationalX,          
            translationalY,
            () -> 0.0,
            () -> 1.0,
            () -> false,
            () -> true,
            redside,
            drivetrain),

        Commands.waitUntil(() -> shooterSubsystem.isAtSpeed() && Targeting.pointingAtHub(redside, drivetrain))

            .andThen(
                hopperSubsystem
                    .runHopperUntilInterruptedCommand()
                    .alongWith(
                        intakeSubsystem
                            .powerRetractRollersCommand()
                            .beforeStarting(
                                Commands.waitSeconds(
                                    Constants.Intake.Arm.POWER_RETRACT_DELAY)))));
  }
}
