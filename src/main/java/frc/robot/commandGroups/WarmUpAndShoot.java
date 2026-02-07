package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class WarmUpAndShoot extends SequentialCommandGroup {
  public WarmUpAndShoot(ShooterSubsystem shooterSubsystem, HopperSubsystem hopperSubsystem) {
    addCommands(
        shooterSubsystem.ShootAtSpeed(),
        hopperSubsystem.runHopperCommand(Constants.Hopper.TARGET_PULLEY_SPEED_M_PER_SEC));
  }
}
