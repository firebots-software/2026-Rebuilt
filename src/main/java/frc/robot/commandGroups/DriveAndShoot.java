package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Shoot;
import frc.robot.commands.SwerveCommands.DriveToPose;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class DriveAndShoot extends ParallelCommandGroup {
  public DriveAndShoot(
      CommandSwerveDrivetrain swerveDriveTrain,
      Supplier<Pose2d> positionToHeadTo,
      ShooterSubsystem shooter,
      HopperSubsystem hopper,
      BooleanSupplier redside) {
    addCommands(
        new Shoot(swerveDriveTrain, shooter, hopper, redside),
        new DriveToPose(swerveDriveTrain, positionToHeadTo).withTargeting(() -> Shoot.running));
  }
}
