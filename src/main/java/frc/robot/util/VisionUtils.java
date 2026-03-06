package frc.robot.util;

import dev.doglog.DogLog;
import frc.robot.Constants;
import frc.robot.Constants.FuelGaugeDetection.FuelGauge;
import frc.robot.Constants.Vision.CameraSelectionMethod;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FuelGaugeDetection;
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
    DogLog.log("Subsystems/Vision/PreferredCamera", preferredVision.getCamera().getLoggingName());
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
}
