package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.BooleanSupplier;

public class BumpDTP extends SequentialCommandGroup {
  public BumpDTP(CommandSwerveDrivetrain swerve, BooleanSupplier forward) {
    // double direction =
    //     (forward.getAsBoolean())
    //         ? (Constants.Swerve.DISTANCE_OVER_BUMP)
    //         : ((-1) * Constants.Swerve.DISTANCE_OVER_BUMP);
    // addCommands(
    //     new DriveToPose(
    //         swerve,
    //         () -> MiscUtils.plus(swerve.getCurrentState().Pose, new Translation2d(direction,
    // 0))));
  }
}
