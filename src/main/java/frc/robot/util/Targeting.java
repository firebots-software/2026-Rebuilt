package frc.robot.util;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.Landmarks;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.MathUtils.Vector3;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Targeting {
  public static class TargetingInfo {
    private double speed;
    private double tof;
    private Vector3 pos;

    public TargetingInfo(double speed, double tof, Vector3 pos) {
      this.speed = speed;
      this.tof = tof;
      this.pos = pos;
    }

    public double getSpeed() {
      return speed;
    }

    public double getToF() {
      return tof;
    }

    public Vector3 getPosition() {
      return pos;
    }
  }

  public static boolean pointingAtTarget(
      Pose3d targetNoOffset, CommandSwerveDrivetrain drivetrain) {
    double desiredRobotHullAngle =
        targetAngle(targetNoOffset, drivetrain) + (2 * Math.PI) % (2 * Math.PI);

    double robotHullAngle =
        drivetrain.getCurrentState().Pose.getRotation().getRadians()
            + (2 * Math.PI) % (2 * Math.PI);

    double diff = Math.abs(desiredRobotHullAngle - robotHullAngle) % (2 * Math.PI);
    if (diff > Math.PI) diff = 2 * Math.PI - diff;
    DogLog.log("Subsystems/Shooter/Shoot/RotationalErrorRadians", diff);
    boolean hullAimed = diff <= Constants.Shooter.ANGULAR_TOLERANCE_FOR_AUTO_AIM_RAD;
    DogLog.log("Subsystems/Shooter/Shoot/Pointing", hullAimed);
    return hullAimed;
  }

  // public static TargetingInfo targetingInfo(
  //     Pose3d target, CommandSwerveDrivetrain drivetrain, int precision) {
  //   Vector3 relativeVel =
  //       Vector3.mult(
  //           new Vector3(
  //               drivetrain.getFieldSpeeds().vxMetersPerSecond,
  //               drivetrain.getFieldSpeeds().vyMetersPerSecond,
  //               0),
  //           -1);

  //   Vector3 shooterPos = new Vector3(drivetrain.getCurrentState().Pose);
  //   Vector3 relativePos = Vector3.subtract(new Vector3(target), shooterPos);

  //   double correctedSpeed = speedForDist(relativePos.magnitude());
  //   double prevTof = 0;
  //   Vector3 correctedPos = new Vector3(target);

  //   for (int i = 0; i < precision; i++) {
  //     double dist = Vector3.subtract(correctedPos, shooterPos).magnitude();
  //     double tof = Constants.Shooter.TOF_FOR_DISTANCE_METERS_CENTER_TO_CENTER_INTERMAP.get(dist);
  //     correctedPos = Vector3.add(correctedPos, Vector3.mult(relativeVel, tof - prevTof));
  //     correctedSpeed = speedForDist(Vector3.subtract(correctedPos, shooterPos).magnitude());
  //     prevTof = tof;
  //   }
  //   DogLog.log("Subsystems/ShooterSubsystem/Shoot/shootspeed", correctedSpeed);
  //   DogLog.log("Subsystems/ShooterSubsystem/Shoot/Tof", prevTof);
  //   DogLog.log("Subsystems/ShooterSubsystem/Shoot/targetPlusLead",
  // Vector3.toPose2d(correctedPos));

  //   return new TargetingInfo(correctedSpeed, prevTof, correctedPos);
  // }

  public static double newtonTargetingDistance(Pose3d target, CommandSwerveDrivetrain swerve) {
    // Load basic stuff in
    ChassisSpeeds currSpeeds = swerve.getFieldSpeeds();
    Pose2d currState = swerve.getPose();

    // initial guess
    double initDX = target.getX() - currState.getX();
    double initDY = target.getY() - currState.getY();
    double initialDistance = Math.pow(initDX * initDX + initDY * initDY, 0.5);

    if (initialDistance < 1e-6) return 0;

    double radialVelocity =
        (initDX * currSpeeds.vxMetersPerSecond + initDY * currSpeeds.vyMetersPerSecond)
            / initialDistance;

    // shit we need
    double tof =
        initialDistance
            / (initialDistance
                    / Constants.Shooter.TOF_FOR_DISTANCE_METERS_CENTER_TO_CENTER_INTERMAP.get(
                        initialDistance)
                - radialVelocity);

    double distance = initialDistance;

    for (int i = 0; i < Constants.Shooter.TARGETING_CALCULATION_PRECISION; i++) {
      double distX = (initDX) - currSpeeds.vxMetersPerSecond * tof;
      double distY = (initDY) - currSpeeds.vyMetersPerSecond * tof;

      distance = Math.pow(distX * distX + distY * distY, 0.5);
      if (distance < 1e-6) break;

      double tofTable =
          Constants.Shooter.TOF_FOR_DISTANCE_METERS_CENTER_TO_CENTER_INTERMAP.get(distance);
      double error = tof - tofTable;

      double horizontalVel = distance / tofTable;
      double errorDerivative =
          1.0
              - ((distX * currSpeeds.vxMetersPerSecond + distY * currSpeeds.vyMetersPerSecond)
                  / (distance * horizontalVel));

      if (Math.abs(errorDerivative) < 1e-3) break;

      if (tof < 1e-3) tof = 1e-3;

      double step = error/errorDerivative;
      step = Math.max(-0.05, Math.min(0.05, step));
      tof -= step;
    }

    return distance;
  }

  public static TargetingInfo newtonTargetingInfo(
      Pose3d target, CommandSwerveDrivetrain drivetrain) {
    double distance = newtonTargetingDistance(target, drivetrain);
    double timeOfFlight =
        Constants.Shooter.TOF_FOR_DISTANCE_METERS_CENTER_TO_CENTER_INTERMAP.get(distance);

    Vector3 relativeVel =
        Vector3.mult(
            new Vector3(
                drivetrain.getFieldSpeeds().vxMetersPerSecond,
                drivetrain.getFieldSpeeds().vyMetersPerSecond,
                0),
            -1);
    Vector3 targetPlusOffset =
        Vector3.add(new Vector3(target), Vector3.mult(relativeVel, timeOfFlight));

    return new TargetingInfo(
        Constants.Shooter.SHOOTER_WHEEL_RPS_FOR_DISTANCE_METERS.get(distance),
        timeOfFlight,
        targetPlusOffset);
  }

  public static double shootingSpeed(
      Pose3d target, CommandSwerveDrivetrain drivetrain, int precision) {
    return newtonTargetingInfo(target, drivetrain).getSpeed();
  }

  public static Vector3 positionToTarget(
      Pose3d target, CommandSwerveDrivetrain drivetrain, int precision) {
    return newtonTargetingInfo(target, drivetrain).getPosition();
  }

  public static double speedForDist(double d) {
    return Constants.Shooter.SHOOTER_WHEEL_RPS_FOR_DISTANCE_METERS.get(d);
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
        Units.metersToFeet(distMeters(drivetrain, target)) > Constants.Shooter.MAX_DIST_FT
                || Units.metersToFeet(distMeters(drivetrain, target))
                    < Constants.Shooter.MIN_DIST_FT
            ? .5d
            : 0d;
  }

  public static boolean pointingAtHub(BooleanSupplier redside, CommandSwerveDrivetrain drivetrain) {
    Pose3d target = redside.getAsBoolean() ? Landmarks.RED_HUB : Landmarks.BLUE_HUB;
    return pointingAtTarget(target, drivetrain);
  }
}
