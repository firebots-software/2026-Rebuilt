package frc.robot.util;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.MathUtils.MiscMath;
import frc.robot.MathUtils.Vector3;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;

public class Targeting {
  public static boolean pointingAtTarget(
      Pose3d targetNoOffset, CommandSwerveDrivetrain drivetrain) {
    double desiredRobotHullAngle =
        (targetAngle(targetNoOffset, drivetrain) + (2 * Math.PI)) % (2 * Math.PI);

    double robotHullAngle =
        (drivetrain.getCurrentState().Pose.getRotation().getRadians() + (2 * Math.PI))
            % (2 * Math.PI);

    double diff = Math.abs(desiredRobotHullAngle - robotHullAngle) % (2 * Math.PI);
    if (diff > Math.PI) diff = 2 * Math.PI - diff;
    DogLog.log("Subsystems/ShooterSubsystem/Shoot/rotationalErrorRadians", diff);
    boolean hullAimed = diff <= Constants.Shooter.ANGULAR_TOLERANCE_FOR_AUTO_AIM_RAD;
    DogLog.log("Subsystems/ShooterSubsystem/Shoot/pointing", hullAimed);
    return hullAimed;
  }

  public static double shootingSpeed(
      Pose3d target, CommandSwerveDrivetrain drivetrain, int precision) { // meters per sec
    Vector3 relativeVel =
        Vector3.mult(
            new Vector3(
                drivetrain.getFieldSpeeds().vxMetersPerSecond,
                drivetrain.getFieldSpeeds().vyMetersPerSecond,
                0),
            -1);

    Pose3d shooterOffset =
        MiscMath.RotatedPosAroundVertical(
            Constants.Shooter.OFFSET_FROM_ROBOT_CENTER,
            drivetrain.getCurrentState().Pose.getRotation().getRadians());
    Vector3 shooterPos =
        Vector3.add(new Vector3(drivetrain.getCurrentState().Pose), new Vector3(shooterOffset));
    Vector3 relativePos = Vector3.subtract(new Vector3(target), shooterPos);

    Vector3 correctedPos = new Vector3(target);
    double correctedSpeed = speedForDist(relativePos.magnitude());
    double prevTof = 0;
    for (int i = 0; i < precision; i++) {
      double tof =
          2
              * correctedSpeed
              * Math.sin(Math.toRadians(Constants.Shooter.SHOOTER_ANGLE_FROM_HORIZONTAL_DEGREES))
              / 9.81;
      correctedPos = Vector3.add(correctedPos, Vector3.mult(relativeVel, tof - prevTof));
      correctedSpeed = speedForDist(Vector3.subtract(correctedPos, shooterPos).magnitude());
      prevTof = tof;
    }
    DogLog.log("Subsystems/ShooterSubsystem/Shoot/shootspeed", correctedSpeed);
    return correctedSpeed;
  }

  public static double speedForDist(double d) {
    return Math.sqrt(
        d
            * 9.81
            / Math.sin(
                Math.toRadians(Constants.Shooter.SHOOTER_ANGLE_FROM_HORIZONTAL_DEGREES) * 2));
  }

  public static Vector3 positionToTarget(
      Pose3d target, CommandSwerveDrivetrain drivetrain, int precision) {
    double timeOfFlight =
        2
            * shootingSpeed(target, drivetrain, precision)
            * Math.sin(Math.toRadians(Constants.Shooter.SHOOTER_ANGLE_FROM_HORIZONTAL_DEGREES))
            / 9.81;

    Vector3 relativeVel =
        Vector3.mult(
            new Vector3(
                drivetrain.getFieldSpeeds().vxMetersPerSecond,
                drivetrain.getFieldSpeeds().vyMetersPerSecond,
                0),
            -1);
    Vector3 targetPlusOffset =
        Vector3.add(new Vector3(target), Vector3.mult(relativeVel, timeOfFlight));
    DogLog.log(
        "Subsystems/ShooterSubsystem/Shoot/targetPlusOffset",
        new Pose2d(targetPlusOffset.x, targetPlusOffset.y, new Rotation2d()));
    return targetPlusOffset;
  }

  public static double targetAngle(Pose3d targetNoOffset, CommandSwerveDrivetrain drivetrain) {
    Vector3 target =
        positionToTarget(
            targetNoOffset, drivetrain, Constants.Shooter.TARGETING_CALCULATION_PRECISION);
    return Math.atan2(
            Vector3.subtract(target, new Vector3(drivetrain.getCurrentState().Pose)).y,
            Vector3.subtract(target, new Vector3(drivetrain.getCurrentState().Pose)).x)
        + (Constants.Shooter.SHOOTS_BACKWARDS ? Math.PI : 0);
  }

  public static double distMeters(CommandSwerveDrivetrain drivetrain, Pose3d target) {
    return Vector3.subtract(new Vector3(drivetrain.getCurrentState().Pose), new Vector3(target))
        .magnitude();
  }

  public static DoubleSupplier amtToRumble(CommandSwerveDrivetrain drivetrain, Pose3d target) {
    return () ->
        (Units.metersToFeet(distMeters(drivetrain, target)) > Constants.Shooter.MAX_DIST_FT
                || Units.metersToFeet(distMeters(drivetrain, target))
                    < Constants.Shooter.MIN_DIST_FT)
            ? .5d
            : 0d;
  }
}
