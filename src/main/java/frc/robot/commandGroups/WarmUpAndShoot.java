package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class WarmUpAndShoot extends SequentialCommandGroup {
  public WarmUpAndShoot(
      DoubleSupplier speed,
      BooleanSupplier readyToShoot,
      ShooterSubsystem shooterSubsystem,
      HopperSubsystem hopperSubsystem) {
    addCommands(
        shooterSubsystem.shootAtSpeedCommand(speed),
        hopperSubsystem
            .runHopperCommand(Constants.Hopper.HOPPER_BELT_TARGET_SPEED_METERS_PER_SECOND)
            .onlyIf(readyToShoot));
  }
}
