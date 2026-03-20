package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.BooleanSupplier;

public class IntakeToBumpDTP extends SequentialCommandGroup {
  public IntakeToBumpDTP(CommandSwerveDrivetrain swerve, BooleanSupplier isRedSide) {
    Pose2d pose =
        isRedSide.getAsBoolean()
            ? new Pose2d(
                new Translation2d(10.908942222595215, 2.54630184173584),
                new Rotation2d(1.5707963267948966))
            : new Pose2d(
                new Translation2d(5.6342058181762695, 5.505496978759766),
                new Rotation2d(-1.5649821399611368));

    // addCommands(new DriveToPoseWithCorrectEndings(swerve, () -> pose));
    addCommands(new DriveToPose(swerve, () -> pose));
  }
}
