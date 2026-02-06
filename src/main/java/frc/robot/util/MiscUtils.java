package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class MiscUtils {
  public static Pose2d plus(Pose2d a, Translation2d b) {
    return new Pose2d(
        a.getX() + b.getX(), a.getY() + b.getY(), new Rotation2d(a.getRotation().getRadians()));
  }

  public static Pose2d plusWithRotation(Pose2d a, Pose2d b) { // Transform2d used to be Pose2d
    return new Pose2d(
        a.getX() + b.getX(),
        a.getY() + b.getY(),
        new Rotation2d(a.getRotation().getRadians() + b.getRotation().getRadians()));
  }
}
