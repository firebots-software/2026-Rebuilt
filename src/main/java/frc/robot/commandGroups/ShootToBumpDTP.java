package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.BooleanSupplier;

public class ShootToBumpDTP extends SequentialCommandGroup {
  public ShootToBumpDTP(
      CommandSwerveDrivetrain swerve, BooleanSupplier isRedSide, BooleanSupplier isLeftSide) {
    Pose2d pose =
        isRedSide.getAsBoolean()
            ? isLeftSide.getAsBoolean()
                ? Constants.Landmarks.RED_LEFT_SHOOT_TO_BUMP
                : Constants.Landmarks.RED_RIGHT_SHOOT_TO_BUMP
            : isLeftSide.getAsBoolean()
                ? Constants.Landmarks.BLUE_LEFT_SHOOT_TO_BUMP
                : Constants.Landmarks.BLUE_RIGHT_SHOOT_TO_BUMP;

    addCommands(new DriveToPose(swerve, () -> pose));
  }
}
