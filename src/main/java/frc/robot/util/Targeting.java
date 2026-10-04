package frc.robot.util;

import dev.doglog.DogLog;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.Units;
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
    private Pose2d pos;

    public TargetingInfo(double speed, double tof, Pose2d pos) {
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

    public Pose2d getPosition() {
      return pos;
    }
  }

  public static Translation2d computeVirtualTarget(
      Pose2d target, CommandSwerveDrivetrain drivetrain) {
    ChassisVelocities fieldSpeeds = drivetrain.getFieldSpeeds();
    Pose2d currPose = drivetrain.getPose();

    // Twist2d twist =
    //     new Twist2d(
    //         fieldSpeeds.vx * 0.03,
    //         fieldSpeeds.vy * 0.03,
    //         fieldSpeeds.omega * 0.03);
    // Pose2d lookaheadPose = currPose.exp(twist);

    double initDX = target.getX() - currPose.getX();
    double initDY = target.getY() - currPose.getY();
    double initialDistance = Math.sqrt(initDX * initDX + initDY * initDY);

    if (initialDistance < 1e-6) return target.getTranslation();

    // double radialVelocity =
    //     (initDX * fieldSpeeds.vx + initDY * fieldSpeeds.vy)
    //         / initialDistance;

    double tof = Constants.Shooter.TIME_OF_FLIGHT_MAP.get(initialDistance);
    // initialDistance
    //     / (initialDistance / Constants.Shooter.TIME_OF_FLIGHT_MAP.get(initialDistance)
    //         - radialVelocity);

    for (int i = 0; i < Constants.Shooter.TARGETING_CALCULATION_PRECISION; i++) {
      double distX = initDX - fieldSpeeds.vx * tof;
      double distY = initDY - fieldSpeeds.vy * tof;
      double distance = Math.sqrt(distX * distX + distY * distY);

      if (distance < 1e-6) break;

      double tofTable = Constants.Shooter.TIME_OF_FLIGHT_MAP.get(distance);
      double error = tof - tofTable;

      if (Math.abs(error) < 1e-3) break;

      double horizontalVel = distance / tofTable;
      double errorDerivative =
          1.0
              + ((distX * fieldSpeeds.vx + distY * fieldSpeeds.vy)
                  / (distance * horizontalVel));

      if (tof < 1e-3) tof = 1e-3;

      double step = error / errorDerivative;
      // step = Math.max(-0.05, Math.min(0.05, step));
      step = Math.max(-0.1 * tof, Math.min(0.1 * tof, step));
      tof -= step;
    }

    return new Translation2d(
        target.getX() - fieldSpeeds.vx * tof,
        target.getY() - fieldSpeeds.vy * tof);
  }

  public static boolean pointingAtTarget(
      Pose2d targetNoOffset, CommandSwerveDrivetrain drivetrain) {
    
    Rotation2d desiredAngle = targetAngle(targetNoOffset, drivetrain);
    Rotation2d currentAngle = drivetrain.getCurrentState().Pose.getRotation();

    double diff = Math.abs(desiredAngle.minus(currentAngle).getRadians());
    
    DogLog.log("Subsystems/Shooter/Shoot/RotationalErrorRadians", diff);
    boolean hullAimed = diff <= Constants.Shooter.ANGULAR_TOLERANCE_FOR_AUTO_AIM_RAD;
    DogLog.log("Subsystems/Shooter/Shoot/Pointing", hullAimed);
    
    return hullAimed;
  }

  public static boolean pointingAtTarget(double angle, CommandSwerveDrivetrain drivetrain) {
    Rotation2d desiredAngle = new Rotation2d(angle);
    Rotation2d currentAngle = drivetrain.getCurrentState().Pose.getRotation();

    DogLog.log("PointingAtTarget/DesiredAngle", desiredAngle.getRadians());
    DogLog.log("PointingAtTarget/RobotAngle", currentAngle.getRadians());
    
    double diff = Math.abs(desiredAngle.minus(currentAngle).getRadians());
    
    return diff <= Constants.Shooter.ANGULAR_TOLERANCE_FOR_AUTO_AIM_RAD;
  }

  // public static TargetingInfo targetingInfo(
  //     Pose2d target, CommandSwerveDrivetrain drivetrain, int precision) {
  //   Vector3 relativeVel =
  //       Vector3.mult(
  //           new Vector3(
  //               drivetrain.getFieldSpeeds().vx,
  //               drivetrain.getFieldSpeeds().vy,
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

  public static double newtonTargetingDistance(Pose2d target, CommandSwerveDrivetrain swerve) {
    // Load basic stuff in
    ChassisVelocities currSpeeds = swerve.getFieldSpeeds();
    Pose2d currState = swerve.getPose();

    // initial guess
    double initDX = target.getX() - currState.getX();
    double initDY = target.getY() - currState.getY();
    double initialDistance = Math.pow(initDX * initDX + initDY * initDY, 0.5);

    if (initialDistance < 1e-6) return 0;

    double radialVelocity =
        (initDX * currSpeeds.vx + initDY * currSpeeds.vy)
            / initialDistance;

    // shit we need
    double tof =
        initialDistance
            / (initialDistance / Constants.Shooter.TIME_OF_FLIGHT_MAP.get(initialDistance)
                - radialVelocity);

    double distance = initialDistance;

    for (int i = 0; i < Constants.Shooter.TARGETING_CALCULATION_PRECISION; i++) {
      double distX = (initDX) - currSpeeds.vx * tof;
      double distY = (initDY) - currSpeeds.vy * tof;

      distance = Math.pow(distX * distX + distY * distY, 0.5);
      if (distance < 1e-6) break;

      double tofTable = Constants.Shooter.TIME_OF_FLIGHT_MAP.get(distance);
      double error = tof - tofTable;

      double horizontalVel = distance / tofTable;
      double errorDerivative =
          1.0
              - ((distX * currSpeeds.vx + distY * currSpeeds.vy)
                  / (distance * horizontalVel));

      if (Math.abs(error) < 1e-3) break;

      if (tof < 1e-3) tof = 1e-3;

      double step = error / errorDerivative;
      step = Math.max(-0.05, Math.min(0.05, step));
      tof -= step;
    }

    return distance;
  }

  public static TargetingInfo newtonTargetingInfo(
      Pose2d target, CommandSwerveDrivetrain drivetrain) {
    double distance = newtonTargetingDistance(target, drivetrain);
    double timeOfFlight = Constants.Shooter.TIME_OF_FLIGHT_MAP.get(distance);

    Vector3 relativeVel =
        Vector3.mult(
            new Vector3(
                drivetrain.getFieldSpeeds().vx,
                drivetrain.getFieldSpeeds().vy,
                0),
            -1);
    Vector3 targetPlusOffset =
        Vector3.add(new Vector3(target), Vector3.mult(relativeVel, timeOfFlight));

    return new TargetingInfo(
        Constants.Shooter.TIME_OF_FLIGHT_MAP.get(distance),
        timeOfFlight,
        Vector3.toPose2d(targetPlusOffset));
  }

  public static double shootingSpeed(Pose2d target, CommandSwerveDrivetrain drivetrain) {
    return newtonTargetingInfo(target, drivetrain).getSpeed();
  }

  public static Pose2d positionToTarget(Pose2d target, CommandSwerveDrivetrain drivetrain) {
    return newtonTargetingInfo(target, drivetrain).getPosition();
  }

  public static double speedForDist(double d) {
    return Constants.Shooter.TIME_OF_FLIGHT_MAP.get(d);
  }

public static Rotation2d targetAngle(Pose2d targetNoOffset, CommandSwerveDrivetrain drivetrain) {
    Pose2d target = positionToTarget(targetNoOffset, drivetrain);
    Translation2d robotToTarget = target.getTranslation().minus(drivetrain.getCurrentState().Pose.getTranslation());
    
    Rotation2d angle = robotToTarget.getAngle().isPresent() ? robotToTarget.getAngle().get() : drivetrain.getCurrentState().Pose.getRotation();
    
    if (Constants.Shooter.SHOOTS_BACKWARDS) {
        return angle.rotateBy(Rotation2d.fromRadians(Math.PI)); 
    }
    
    return angle;
  }

  public static double distMeters(CommandSwerveDrivetrain drivetrain, Pose2d target) {
    return Vector3.subtract(new Vector3(drivetrain.getCurrentState().Pose), new Vector3(target))
        .magnitude();
  }

  public static DoubleSupplier amtToRumble(CommandSwerveDrivetrain drivetrain, Pose2d target) {
    return () ->
        Units.metersToFeet(distMeters(drivetrain, target)) > Constants.Shooter.MAX_DIST_FT
                || Units.metersToFeet(distMeters(drivetrain, target))
                    < Constants.Shooter.MIN_DIST_FT
            ? .5d
            : 0d;
  }

  public static boolean pointingAtHub(BooleanSupplier redside, CommandSwerveDrivetrain drivetrain) {
    Pose2d target = redside.getAsBoolean() ? Landmarks.RED_HUB : Landmarks.BLUE_HUB;
    return pointingAtTarget(target, drivetrain);
  }

  public static Pose2d getHub(BooleanSupplier redside) {
    return (redside.getAsBoolean() ? Landmarks.RED_HUB : Landmarks.BLUE_HUB);
  }
}
