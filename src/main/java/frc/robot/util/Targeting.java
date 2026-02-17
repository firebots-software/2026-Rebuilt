package frc.robot.util;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;
import frc.robot.MathUtils.MiscMath;
import frc.robot.MathUtils.Vector3;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Targeting {
  public static boolean pointingAtTarget(
      Pose3d targetNoOffset, CommandSwerveDrivetrain drivetrain) {
    double desiredRobotHullAngle = targetAngle(targetNoOffset, drivetrain);

    double robotHullAngle =
        (drivetrain.getState().Pose.getRotation().getRadians() + (2 * Math.PI)) % (2 * Math.PI);

    DogLog.log(
        "Subsystems/ShooterSubsystem/Shoot/rotationalErrorRadians",
        Math.abs(desiredRobotHullAngle - robotHullAngle));
    boolean hullAimed =
        Math.abs(desiredRobotHullAngle - robotHullAngle)
            <= Constants.Shooter.ANGULAR_TOLERANCE_FOR_AUTO_AIM_RAD;
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

    Pose3d gunOffset =
        MiscMath.RotatedPosAroundVertical(
            Constants.Shooter.OFFSET_FROM_ROBOT_CENTER,
            drivetrain.getState().Pose.getRotation().getRadians());
    Vector3 gunPos = Vector3.add(new Vector3(drivetrain.getState().Pose), new Vector3(gunOffset));
    Vector3 relativePos = Vector3.subtract(new Vector3(target), gunPos);

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
      correctedSpeed = speedForDist(Vector3.subtract(correctedPos, gunPos).magnitude());
      prevTof = tof;
    }

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

    return Vector3.add(new Vector3(target), Vector3.mult(relativeVel, timeOfFlight));
  }

  public static double targetAngle(Pose3d targetNoOffset, CommandSwerveDrivetrain drivetrain) {
    Vector3 target =
        positionToTarget(
            targetNoOffset, drivetrain, Constants.Shooter.TARGETING_CALCULATION_PRECISION);
    return Math.atan2(
            Vector3.subtract(target, new Vector3(drivetrain.getState().Pose)).y,
            Vector3.subtract(target, new Vector3(drivetrain.getState().Pose)).x)
        + (Constants.Shooter.SHOOTS_BACKWARDS ? Math.PI : 0);
  }
}
