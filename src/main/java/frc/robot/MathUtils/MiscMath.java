package frc.robot.MathUtils;

import edu.wpi.first.math.geometry.Pose3d;

public class MiscMath {
  public static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  public static Pose3d RotatedPosAroundVertical(Pose3d pose, double r) {
    return new Pose3d(
        pose.getX() * Math.cos(r) - pose.getY() * Math.sin(r),
        pose.getX() * Math.sin(r) + pose.getY() * Math.cos(r),
        pose.getZ(),
        pose.getRotation());
  }

  public static double computeShootingSpeed(double distToHubCenter) {
  // Constants (meters)
  final double a = edu.wpi.first.math.util.Units.inchesToMeters(5.67405);
  final double b = edu.wpi.first.math.util.Units.inchesToMeters(36.60021);

  double y = (a * Math.sqrt(distToHubCenter)) + b;

  return clamp(y, 71.0, 107.0);
}
}
