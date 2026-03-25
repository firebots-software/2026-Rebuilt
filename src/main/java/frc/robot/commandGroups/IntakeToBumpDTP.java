package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.BooleanSupplier;

public class IntakeToBumpDTP extends SequentialCommandGroup {
  public IntakeToBumpDTP(CommandSwerveDrivetrain swerve, BooleanSupplier isRedSide) {
    Pose2d pose =
        isRedSide.getAsBoolean()
            ? Constants.Landmarks.RED_INTAKE_TO_BUMP
            : Constants.Landmarks.BLUE_INTAKE_TO_BUMP;

    addCommands(new DriveToPose(swerve, () -> pose));
  }
}
