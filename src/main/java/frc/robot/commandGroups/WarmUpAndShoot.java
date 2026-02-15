package frc.robot.commandGroups;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class WarmUpAndShoot extends SequentialCommandGroup {
  public WarmUpAndShoot(double speed, BooleanSupplier readyToShoot, ShooterSubsystem shooterSubsystem, HopperSubsystem hopperSubsystem) {
    addCommands(
        shooterSubsystem.ShootAtSpeed(speed),
        hopperSubsystem.runHopperCommand(Constants.Hopper.TARGET_PULLEY_SPEED_M_PER_SEC).onlyIf(readyToShoot));
  }
}