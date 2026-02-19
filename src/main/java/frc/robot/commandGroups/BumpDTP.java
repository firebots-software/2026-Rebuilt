package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class BumpDTP extends SequentialCommandGroup {
    public BumpDTP(CommandSwerveDrivetrain drive, double distanceMeters) {
        addCommands(new DriveToPose(drive, () -> {
            Pose2d currentPose = drive.getPose();

            Transform2d forward = new Transform2d(new Translation2d(distanceMeters, 0), new Rotation2d());

            return currentPose.plus(forward);
        }));
    }
}
