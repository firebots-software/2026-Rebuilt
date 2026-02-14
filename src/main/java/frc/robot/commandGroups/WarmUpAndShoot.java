package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class WarmUpAndShoot extends SequentialCommandGroup {
  public WarmUpAndShoot(ShooterSubsystem shooterSubsystem, HopperSubsystem hopperSubsystem) {
    addCommands(
        shooterSubsystem.shootAtSpeed(),
        hopperSubsystem.runHopperCommand(
            Constants.Hopper.HOPPER_BELT_TARGET_SPEED_METERS_PER_SECOND));
  }
}
