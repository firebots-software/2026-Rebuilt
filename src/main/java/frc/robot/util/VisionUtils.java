package frc.robot.util;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.FuelGaugeDetection.FuelGauge;
import frc.robot.Constants.Vision.CameraSelectionMethod;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FuelGaugeDetection;
import frc.robot.subsystems.IntakeVisionDetection;
import frc.robot.subsystems.VisionSubsystem;

public class VisionUtils {

  private static VisionSubsystem visionFrontRight, visionFrontLeft, visionRearRight, visionRearLeft;
  private static VisionSubsystem preferredVision;
  private static CommandSwerveDrivetrain drivetrain;

  public static void visionPeriodic(
      VisionSubsystem frontRight,
      VisionSubsystem frontLeft,
      VisionSubsystem rearRight,
      VisionSubsystem rearLeft,
      CommandSwerveDrivetrain swerve) {

    if (!Constants.visionOnRobot
        || frontRight == null
        || frontLeft == null
        || rearRight == null
        || rearLeft == null) return;

    visionFrontRight = frontRight;
    visionFrontLeft = frontLeft;
    visionRearRight = rearRight;
    visionRearLeft = rearLeft;
    drivetrain = swerve;

    calculateAllCameraPoses();

    preferredVision = getVisionFallback();

    if (!Constants.Vision.SKIP_TO_FALLBACK) preferredVision = selectPreferredVision();

    if (preferredVision == null || !preferredVision.hasValidMeasurement()) return;

    preferredVision.addFilteredPose(drivetrain);

    preferredVisionLogs();
  }

  private static VisionSubsystem getVisionFallback() {
    switch (Constants.Vision.FALLBACK_CAMERA) {
      case FRONT_RIGHT_CAM:
        return visionFrontRight;
      case FRONT_LEFT_CAM:
      default:
        return visionFrontLeft;
      case REAR_RIGHT_CAM:
        return visionRearRight;
      case REAR_LEFT_CAM:
        return visionRearLeft;
    }
  }

  private static void calculateAllCameraPoses() {
    visionFrontRight.calculateFilteredPose(drivetrain);
    visionFrontLeft.calculateFilteredPose(drivetrain);
    visionRearRight.calculateFilteredPose(drivetrain);
    visionRearLeft.calculateFilteredPose(drivetrain);
  }

  private static VisionSubsystem selectPreferredVision() {

    CameraSelectionMethod method = Constants.Vision.CAMERA_SELECTION_METHOD;
    double frontRightDist = getCameraDistance(visionFrontRight, method);
    double frontLeftDist = getCameraDistance(visionFrontLeft, method);
    double rearRightDist = getCameraDistance(visionRearRight, method);
    double rearLeftDist = getCameraDistance(visionRearLeft, method);

    return compareCameraDistance(frontRightDist, frontLeftDist, rearRightDist, rearLeftDist);
  }

  private static double getCameraDistance(
      VisionSubsystem vision, Constants.Vision.CameraSelectionMethod method) {
    switch (method) {
      case MIN:
      default:
        return vision.getMinDistance();
      case AVG:
        return vision.getAverageDistance();
      case MAX:
        return vision.getMaxDistance();
      case POSE_AMBIGUITY:
        return vision.getPoseAmbiguity();
    }
  }

  private static VisionSubsystem compareCameraDistance(
      double frontRight, double frontLeft, double rearRight, double rearLeft) {
    double bestDistance = Double.MAX_VALUE;
    VisionSubsystem bestVision = preferredVision;

    if (frontRight < bestDistance && visionFrontRight.hasValidMeasurement()) {
      bestVision = visionFrontRight;
      bestDistance = frontRight;
    }

    if (frontLeft < bestDistance && visionFrontLeft.hasValidMeasurement()) {
      bestVision = visionFrontLeft;
      bestDistance = frontLeft;
    }

    if (rearRight < bestDistance && visionRearRight.hasValidMeasurement()) {
      bestVision = visionRearRight;
      bestDistance = rearRight;
    }

    if (rearLeft < bestDistance && visionRearLeft.hasValidMeasurement()) {
      bestVision = visionRearLeft;
      bestDistance = rearLeft;
    }

    return bestVision;
  }

  private static void preferredVisionLogs() {
    DogLog.log("Subsystems/Vision/PreferredCamera", preferredVision.getCameraID().getLoggingName());
    DogLog.log("Subsystems/Vision/CompletePoseEstimate", drivetrain.getState().Pose);
    DogLog.log("Subsystems/Vision/RawPoseEstimate", preferredVision.getFilteredPose());
  }

  public static void fuelGaugeLogs(FuelGaugeDetection visionFuelGauge) {
    if (Constants.fuelGaugeOnRobot && visionFuelGauge != null) {
      FuelGauge gaugeState = visionFuelGauge.getCurrentFuelGaugeState();
      DogLog.log("Elastic/FuelGauge", gaugeState.toString());
      DogLog.log("Elastic/FuelGauge/CameraConnected", true);
    } else {
      DogLog.log("Elastic/FuelGauge", "N/A");
      DogLog.log("Elastic/FuelGauge/CameraConnected", false);
    }
  }

  public static String getColorOrDefault(FuelGauge gauge) {
    return gauge == null ? "#FFFFFF" : gauge.getColor();
  }

  public static Matrix<N3, N1> computeNoiseVector(double distance, double speed, int tagCount) {
    double nX = computeNoiseX(distance, speed, tagCount);
    double nY = computeNoiseY(distance, speed, tagCount);
    double nTH = computeNoiseHeading(distance, speed, tagCount);
    return VecBuilder.fill(nX, nY, nTH);
  }

  private static double computeNoiseXY(
      double baseNoise,
      double distanceExponentialCoefficient,
      double distanceExponentialBase,
      double angleCoefficient,
      double speedCoefficient,
      double distance,
      double robotSpeed,
      int tagCount) {

    double maximumRobotSpeed = Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;

    // Tag count factor (cap at 4 - diminishing returns)
    int effectiveTags = Math.min(tagCount, 4);
    double tagFactor = 1d / Math.sqrt(effectiveTags);

    double distanceFactor =
        baseNoise + distanceExponentialCoefficient * Math.pow(distanceExponentialBase, distance);

    // Speed term (quadratic)
    double vNorm = Math.min(robotSpeed, maximumRobotSpeed) / maximumRobotSpeed;
    double speedFactor = 1d + speedCoefficient * (vNorm * vNorm);

    DogLog.log("Subsystems/Vision/calibrationFactor", Constants.Vision.CALIBRATION_FACTOR);
    DogLog.log("Subsystems/Vision/tagFactor", tagFactor);
    DogLog.log("Subsystems/Vision/distanceFactor", distanceFactor);
    DogLog.log("Subsystems/Vision/speedFactor", speedFactor);

    double computedStdDevs =
        Constants.Vision.CALIBRATION_FACTOR * tagFactor * distanceFactor * speedFactor;
    return computedStdDevs;
  }

  private static double computeNoiseHeading(double distance, double robotSpeed, int tagCount) {

    double baseNoise = Constants.Vision.BASE_NOISE_THETA;
    double distanceCoefficient = Constants.Vision.DISTANCE_COEFFICIENT_THETA;
    double angleCoefficient = Constants.Vision.ANGLE_COEFFICIENT_THETA;
    double speedCoefficient = Constants.Vision.SPEED_COEFFICIENT_THETA;
    double maximumRobotSpeed = Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;

    // Tag count factor (cap at 4 - diminishing returns)
    int effectiveTags = Math.min(tagCount, 4);
    double tagFactor = 1d / Math.sqrt(effectiveTags);

    // Distance term (d^2)
    double distanceFactor = baseNoise + distanceCoefficient * distance * distance;

    double vNorm = Math.min(robotSpeed, maximumRobotSpeed) / maximumRobotSpeed;
    double speedFactor = 1d + speedCoefficient * (vNorm * vNorm);

    double computedStdDevs =
        Constants.Vision.CALIBRATION_FACTOR * tagFactor * distanceFactor * speedFactor;
    return computedStdDevs;
  }

  private static double computeNoiseX(double distance, double robotSpeed, int tagCount) {
    return computeNoiseXY(
        Constants.Vision.BASE_NOISE_X,
        Constants.Vision.DISTANCE_EXPONENTIAL_COEFFICIENT_X,
        Constants.Vision.DISTANCE_EXPONENTIAL_BASE_X,
        Constants.Vision.ANGLE_COEFFICIENT_X,
        Constants.Vision.SPEED_COEFFICIENT_X,
        distance,
        robotSpeed,
        tagCount);
  }

  private static double computeNoiseY(double distance, double robotSpeed, int tagCount) {
    return computeNoiseXY(
        Constants.Vision.BASE_NOISE_Y,
        Constants.Vision.DISTANCE_EXPONENTIAL_COEFFICIENT_Y,
        Constants.Vision.DISTANCE_EXPONENTIAL_BASE_Y,
        Constants.Vision.ANGLE_COEFFICIENT_Y,
        Constants.Vision.SPEED_COEFFICIENT_Y,
        distance,
        robotSpeed,
        tagCount);
  }

  public static Pose2d intakeVisionTargetPose(
      Pose2d currentPose, IntakeVisionDetection intakeVision) {
    Rotation2d rotate = new Rotation2d(Units.degreesToRadians(-intakeVision.getYaw()));
    Translation2d translate = new Translation2d(0, rotate);

    Transform2d poseManipulation = new Transform2d(translate, rotate);
    return currentPose.plus(poseManipulation);
  }
}
