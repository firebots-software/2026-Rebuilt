package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.MiscUtils;

public class BackBumpDTP extends SequentialCommandGroup {
  public BackBumpDTP(CommandSwerveDrivetrain swerve) {
    addCommands(
        new DriveToPose(
            swerve,
            () ->
                MiscUtils.plus(
                    swerve.getCurrentState().Pose,
                    new Translation2d(-1 * Constants.Swerve.DISTANCE_OVER_BUMP, 0))));
  }
}
