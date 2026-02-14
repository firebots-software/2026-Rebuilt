package frc.robot.commandGroups;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.BooleanSupplier;

public class ShootWithWarning extends ParallelCommandGroup {
  public ShootWithWarning(
      CommandSwerveDrivetrain swerveDriveTrain,
      ShooterSubsystem shooter,
      HopperSubsystem hopper,
      BooleanSupplier redside,
      Joystick joystick) {
    addCommands(
        new Shoot(swerveDriveTrain, shooter, hopper, redside),
        new InstantCommand(
            () ->
                joystick.setRumble(
                    RumbleType.kBothRumble,
                    (Shoot.distMeters > Constants.Shooter.MAX_DIST_FT
                            || Shoot.distMeters < Constants.Shooter.MIN_DIST_FT)
                        ? .5d
                        : 0d)));
  }
}
