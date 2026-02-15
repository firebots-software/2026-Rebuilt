package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.MathUtils.Vector3;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ShootWithWarning extends ParallelCommandGroup {
  public ShootWithWarning(
      CommandSwerveDrivetrain swerveDriveTrain,
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      HopperSubsystem hopper,
      DoubleSupplier tangentialVel,
      Pose3d target,
      BooleanSupplier redside,
      Joystick joystick) {

    double distMeters = Vector3.subtract(new Vector3(swerveDriveTrain.getCurrentState().Pose), new Vector3(target)).magnitude();
    addCommands(
        new ArcAroundAndShoot(swerveDriveTrain, shooter, intake, hopper, tangentialVel, target, redside),
        new InstantCommand(
            () ->
                joystick.setRumble(
                    RumbleType.kBothRumble,
                    (distMeters > Constants.Shooter.MAX_DIST_FT
                            || distMeters < Constants.Shooter.MIN_DIST_FT)
                        ? .5d
                        : 0d)));
  }
}
