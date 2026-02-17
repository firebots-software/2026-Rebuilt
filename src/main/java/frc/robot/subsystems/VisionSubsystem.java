package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {

  // static member that contains array of all VisionSubsytem cameras
  private static VisionSubsystem[] cameraList =
      new VisionSubsystem[Constants.Vision.Cameras.values().length];

  private final Constants.Vision.Cameras cameraID;

  private String cameraTitle;

  // list of all april tags, not sorted by red/blue alliance due to neccessity of
  // accessing both
  private static final List<Integer> TAG_IDS =
      List.of(
          1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
          26, 27, 28, 29, 30, 31, 32);

  // NOTE FOR SID/SAKETH: come back to ln 57-70 in 2025 repo

  // VISION:
  private double maxDistance = 15.0; // meters, beyond which readings are dropped

  // normalization maximums
  private double maximumRobotSpeed = 5d;

  // Noise parameters
  private double calibrationFactor = 1d; // constant multiplier to everything
  private double baseNoiseX = 0.0008; // meters
  private double baseNoiseY = 0.0008;
  private double baseNoiseTheta = 0.5; // radians

  // references for PhotonVision
  private final PhotonCamera photonCamera;
  private final PhotonPoseEstimator poseEstimator;
  private PhotonPipelineResult latestVisionResult;
  private Optional<MultiTargetPNPResult> visionResult;
  Optional<EstimatedRobotPose> visionEst;
  private List<PhotonTrackedTarget> tags;
  private final AprilTagFieldLayout fieldLayout;

  public final double acceptableYawThreshold = 60d;

  public static final double timestampDiffThreshold = 0.5;
  public static final double timestampFPGACorrection = -0.03;

  // constructor for VisionSubsystem
  public VisionSubsystem(Constants.Vision.Cameras cameraID) {
    this.cameraID = cameraID;
    photonCamera = new PhotonCamera(cameraID.toString());
    Transform3d robotToCamera = Constants.Vision.getCameraTransform(cameraID);

    // load field layout
    this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // initialize poseEstimator

    poseEstimator = new PhotonPoseEstimator(fieldLayout, robotToCamera);

    cameraTitle = cameraID.getLoggingName();
    latestVisionResult = null;
  }

  public static VisionSubsystem getInstance(Constants.Vision.Cameras cameraID) {
    int index = cameraID.ordinal();
    if (cameraList[index] == null) cameraList[index] = new VisionSubsystem(cameraID);
    return cameraList[index];
  }

  @Override
  public void periodic() {

    // VISION:
    visionEst = Optional.empty();
    latestVisionResult = null;

    List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();
    for (PhotonPipelineResult result : results) {
      latestVisionResult = result;
      visionEst = poseEstimator.estimateCoprocMultiTagPose(result);
      if (visionEst.isEmpty()) {
        visionEst = poseEstimator.estimateLowestAmbiguityPose(result);
      }
    }

    DogLog.log("Subsystems/Vision/" + cameraTitle + "/CameraConnected", true);
  }

  public void addFilteredPose(CommandSwerveDrivetrain swerve) {
    DogLog.log("Subsystems/Vision/addFilteredPoseworking", true);

    if (latestVisionResult == null || latestVisionResult.getTargets().isEmpty()) {
      DogLog.log("Subsystems/Vision/" + cameraTitle + "/HasEstimate", visionEst.isPresent());
      DogLog.log("Subsystems/Vision/" + cameraTitle + "/HasEstimate", false);
      return;
    }
    DogLog.log("Subsystems/Vision/" + cameraTitle + "/HasEstimate", visionEst.isPresent());
    DogLog.log("Subsystems/Vision/" + cameraTitle + "/HasEstimate", true);

    // Ensure we have a valid pose estimate and vision result from periodic()
    if (visionEst.isEmpty()) {
      return;
    }

    // distance to closest april tag
    double minDistance =
        latestVisionResult.getTargets().stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .min()
            .orElse(Double.NaN);

    DogLog.log("Subsystems/Vision/closestTagDistance", minDistance);

    // average distance to all visible april tags
    double averageDistance =
        latestVisionResult.getTargets().stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(Double.NaN);
    DogLog.log("Subsystems/Vision/averageTagDistance", averageDistance);

    // 2025-reefscape has a validTags list on lines 160-166, replacing it with a
    // list of all tags
    // for 26

    // VISION: to end
    // creates a list of all detected tags and logs for debugging
    List<PhotonTrackedTarget> tags =
        latestVisionResult.getTargets().stream().collect(Collectors.toList());

    // log area and yaw for all detected april tags
    for (PhotonTrackedTarget tag : tags) {
      DogLog.log("Subsystems/Vision/" + cameraTitle + "/Area", tag.getArea());
      DogLog.log("Subsystems/Vision/" + cameraTitle + "/Yaw", tag.getYaw());
    }
    // Extract pose estimate
    EstimatedRobotPose estimatedPose = visionEst.get();
    Pose2d measuredPose = estimatedPose.estimatedPose.toPose2d();
    DogLog.log("Subsystems/Vision/" + cameraTitle + "/MeasuredPose", measuredPose);

    // Get detected tags
    tags = latestVisionResult.getTargets();
    if (tags.isEmpty()) {
      DogLog.log("Subsystems/Vision/" + cameraTitle + "/Tags", false);
      return;
    }
    DogLog.log("Subsystems/Vision/" + cameraTitle + "/Tags", true);

    // Distance calculations
    minDistance =
        tags.stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .min()
            .orElse(Double.NaN);

    averageDistance =
        tags.stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(Double.NaN);

    DogLog.log("Subsystems/Vision/closestTagDistance", minDistance);
    DogLog.log("Subsystems/Vision/averageTagDistance", averageDistance);

    // Reject invalid or distant measurements
    if (Double.isNaN(minDistance) || minDistance > maxDistance) {
      DogLog.log("Subsystems/Vision/" + cameraTitle + "/ThrownOutDistance", true);
      return;
    }
    DogLog.log("Subsystems/Vision/" + cameraTitle + "/ThrownOutDistance", false);

    // Log yaw + area for debugging
    for (PhotonTrackedTarget tag : tags) {
      DogLog.log("Subsystems/Vision/" + cameraTitle + "/TagYaw", tag.getYaw());
      DogLog.log("Subsystems/Vision/" + cameraTitle + "/TagArea", tag.getArea());
    }

    int tagCount = tags.size();
    DogLog.log("Subsystems/Vision/tagCount", tagCount);

    // TODO: Replace with real swerve speed
    double currentSpeed = 0.0;

    // Compute noise model
    double nX =
        computeNoiseXY(
            baseNoiseX,
            Constants.Vision.DISTANCE_EXPONENTIAL_COEFFICIENT_X,
            Constants.Vision.DISTANCE_EXPONENTIAL_BASE_X,
            Constants.Vision.ANGLE_COEFFICIENT_X,
            Constants.Vision.SPEED_COEFFICIENT_X,
            averageDistance,
            currentSpeed,
            tagCount);

    double nY =
        computeNoiseXY(
            baseNoiseY,
            Constants.Vision.DISTANCE_EXPONENTIAL_COEFFICIENT_Y,
            Constants.Vision.DISTANCE_EXPONENTIAL_BASE_Y,
            Constants.Vision.ANGLE_COEFFICIENT_Y,
            Constants.Vision.SPEED_COEFFICIENT_Y,
            averageDistance,
            currentSpeed,
            tagCount);

    double nTH =
        computeNoiseHeading(
            baseNoiseTheta,
            Constants.Vision.DISTANCE_COEFFICIENT_THETA,
            Constants.Vision.ANGLE_COEFFICIENT_THETA,
            Constants.Vision.SPEED_COEFFICIENT_THETA,
            averageDistance,
            currentSpeed,
            tagCount);

    Matrix<N3, N1> noiseVector = VecBuilder.fill(nX, nY, nTH);

    // Send to pose estimator / swerve
    processPoseEstimate(
        measuredPose,
        averageDistance,
        currentSpeed,
        tagCount,
        estimatedPose.timestampSeconds,
        noiseVector,
        swerve);

    DogLog.log("Subsystems/Vision/VisionPoseEstimate", measuredPose);

    if (measuredPose == null) {
      DogLog.log("Subsystems/Vision/measuredPoseAvailable", false);
    } else {
      DogLog.log("Subsystems/Vision/measuredPoseAvailable", true);
    }
  }

  private void processPoseEstimate(
      Pose2d measuredPose,
      double averageDistance,
      double currentSpeed,
      int tagCount,
      double timestamp,
      Matrix<N3, N1> noiseVector,
      CommandSwerveDrivetrain swerve) {

    // Use vision timestamp if within threshold of FPGA timestamp
    double fpgaTimestamp = Timer.getFPGATimestamp();
    double timestampDiff = Math.abs(timestamp - fpgaTimestamp);
    double finalTimestamp =
        (timestampDiff > timestampDiffThreshold)
            ? fpgaTimestamp + timestampFPGACorrection
            : timestamp;

    swerve.addVisionMeasurement(measuredPose, finalTimestamp, noiseVector);
  }

  // prev. year had isTagOnActiveSide but not relevant this year

  private boolean acceptableYaw(double yaw) {
    boolean yawIsAcceptable = Math.abs(yaw) < acceptableYawThreshold;
    DogLog.log("Subsystems/Vision/acceptableYaw", yawIsAcceptable);

    return yawIsAcceptable;
  }

  private double computeNoiseXY(
      double baseNoise,
      double distanceExponentialCoefficient,
      double distanceExponentialBase,
      double angleCoefficient,
      double speedCoefficient,
      double distance,
      double robotSpeed,
      int tagCount) {

    // Tag count factor (cap at 4 - diminishing returns)
    int effectiveTags = Math.min(tagCount, 4);
    double tagFactor = 1d / Math.sqrt(effectiveTags);

    // distance term (d^2)
    // Changed from last year: Purely distrust based on distance (as opposed to
    // capping distrust for
    // closer tags)
    // double distanceFactor =
    // baseNoise + distanceExponentialCoefficient *
    // Math.pow(distanceExponentialBase, distance);

    double distanceFactor =
        (distance < (17.548 + 0.67))
            ? Math.min(
                baseNoise
                    + distanceExponentialCoefficient * Math.pow(distanceExponentialBase, distance),
                1.167)
            : (baseNoise
                + distanceExponentialCoefficient * Math.pow(distanceExponentialBase, distance));

    // Speed term (quadratic)
    double vNorm = Math.min(robotSpeed, maximumRobotSpeed) / maximumRobotSpeed;
    double speedFactor = 1d + speedCoefficient * (vNorm * vNorm);

    DogLog.log("Subsystems/Vision/calibrationFactor", calibrationFactor);
    DogLog.log("Subsystems/Vision/tagFactor", tagFactor);
    DogLog.log("Subsystems/Vision/distanceFactor", distanceFactor);
    DogLog.log("Subsystems/Vision/speedFactor", speedFactor);

    double computedStdDevs = calibrationFactor * tagFactor * distanceFactor * speedFactor;
    return computedStdDevs;
  }

  private double computeNoiseHeading(
      double baseNoise,
      double distanceCoefficient,
      double angleCoefficient,
      double speedCoefficient,
      double distance,
      double robotSpeed,
      int tagCount) {

    // Tag count factor (cap at 4 - diminishing returns)
    int effectiveTags = Math.min(tagCount, 4);
    double tagFactor = 1d / Math.sqrt(effectiveTags);

    // Distance term (d^2)
    double distanceFactor = baseNoise + distanceCoefficient * distance * distance;

    double vNorm = Math.min(robotSpeed, maximumRobotSpeed) / maximumRobotSpeed;
    double speedFactor = 1d + speedCoefficient * (vNorm * vNorm);

    double computedStdDevs = calibrationFactor * tagFactor * distanceFactor * speedFactor;
    return computedStdDevs;
  }
}
