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
import java.util.function.BooleanSupplier;
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

  // Camera datatype with only 2 options, left or right
  private final Constants.Vision.Cameras cameraID;

  private String cameraTitle;

  // list of all april tags, not sorted by red/blue alliance due to neccessity of accessing both
  private static final List<Integer> TAG_IDS =
      List.of(
          1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
          26, 27, 28, 29, 30, 31, 32);

  // NOTE FOR SID/SAKETH: come back to ln 57-70 in 2025 repo

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
  private final BooleanSupplier isRedSide;
  private final AprilTagFieldLayout fieldLayout;

  public final double acceptableYawThreshold = 60d;

  public static final double timestampDiffThreshold = 0.5;
  public static final double timestampFPGACorrection = -0.03;

  // constructor for VisionSubsystem
  public VisionSubsystem(Constants.Vision.Cameras cameraID, BooleanSupplier isRedSide) {
    this.isRedSide = isRedSide;
    this.cameraID = cameraID;
    photonCamera = new PhotonCamera(cameraID.toString());
    Transform3d cameraToRobot = Constants.Vision.getCameraTransform(cameraID);

    // load field layout
    this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // initialize poseEstimator - changes on ln 77 (breaks on new), 177
    poseEstimator = new PhotonPoseEstimator(fieldLayout, cameraToRobot);
    if (poseEstimator != null) {
      DogLog.log("Vision/PoseEstimator", true);
      return;
    }
    DogLog.log("Vision/PoseEstimator", false);

    cameraTitle = cameraID.getLoggingName();
    latestVisionResult = null;
  }

  // returns VisionSubsystem instance
  public static VisionSubsystem getInstance(
      Constants.Vision.Cameras cameraID, BooleanSupplier isRedSide) {
    int index = cameraID.ordinal();
    if (cameraList[index] == null) cameraList[index] = new VisionSubsystem(cameraID, isRedSide);
    return cameraList[index];
  }

  @Override
  public void periodic() {
    visionEst = Optional.empty();
    latestVisionResult = null;

    for (PhotonPipelineResult result : photonCamera.getAllUnreadResults()) {
      latestVisionResult = result;
      visionEst = poseEstimator.estimateCoprocMultiTagPose(result);
      if (visionEst.isEmpty()) {
        visionEst = poseEstimator.estimateLowestAmbiguityPose(result);
      }
    }

    DogLog.log("Vision/" + cameraTitle + "/HasEstimate", visionEst.isPresent());
  }

  public void addFilteredPose() {
    // Ensure we have a valid pose estimate and vision result from periodic()
    if (visionEst.isEmpty() || latestVisionResult == null) {
      DogLog.log("Vision/" + cameraTitle + "/HasTargets", false);
      return;
    }
    DogLog.log("Vision/" + cameraTitle + "/HasTargets", true);

    // Extract pose estimate
    EstimatedRobotPose estimatedPose = visionEst.get();
    Pose2d measuredPose = estimatedPose.estimatedPose.toPose2d();
    DogLog.log("Vision/MeasuredPose", measuredPose);

    // Get detected tags
    tags = latestVisionResult.getTargets();
    if (tags.isEmpty()) {
      DogLog.log("Vision/" + cameraTitle + "/Tags", false);
      return;
    }
    DogLog.log("Vision/" + cameraTitle + "/Tags", true);

    // Distance calculations
    double minDistance =
        tags.stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .min()
            .orElse(Double.NaN);

    double averageDistance =
        tags.stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(Double.NaN);

    DogLog.log("Vision/closestTagDistance", minDistance);
    DogLog.log("Vision/averageTagDistance", averageDistance);

    // Reject invalid or distant measurements
    if (Double.isNaN(minDistance) || minDistance > maxDistance) {
      DogLog.log("Vision/" + cameraTitle + "/ThrownOutDistance", true);
      return;
    }
    DogLog.log("Vision/" + cameraTitle + "/ThrownOutDistance", false);

    // Log yaw + area for debugging
    for (PhotonTrackedTarget tag : tags) {
      DogLog.log("Vision/" + cameraTitle + "/TagYaw", tag.getYaw());
      DogLog.log("Vision/" + cameraTitle + "/TagArea", tag.getArea());
    }

    int tagCount = tags.size();
    DogLog.log("Vision/tagCount", tagCount);

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
        noiseVector);

    DogLog.log("Vision/VisionPoseEstimate", measuredPose);

    if (measuredPose == null) {
      DogLog.log("Vision/measuredPoseAvailable", false);
    } else {
      DogLog.log("Vision/measuredPoseAvailable", true);
    }
  }

  // to be completed; method aims to combine final pose estimate with odometry for accurate
  // estimation
  private void processPoseEstimate(
      Pose2d measuredPose,
      double averageDistance,
      double currentSpeed,
      int tagCount,
      double timestamp,
      Matrix<N3, N1> noiseVector) {

    // Use vision timestamp if within threshold of FPGA timestamp
    double fpgaTimestamp = Timer.getFPGATimestamp();
    double timestampDiff = Math.abs(timestamp - fpgaTimestamp);
    double chosenTimestamp =
        (timestampDiff > timestampDiffThreshold)
            ? fpgaTimestamp + timestampFPGACorrection
            : timestamp;

    // swerveDrive.addVisionMeasurement(measuredPose...) check ln 279
  }

  // prev. year had isTagOnActiveSide but not relevant this year

  private boolean acceptableYaw(double yaw) {
    boolean yawIsAcceptable = Math.abs(yaw) < acceptableYawThreshold;
    DogLog.log("Vision/acceptableYaw", yawIsAcceptable);

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
    // Changed from last year: Purely distrust based on distance (as opposed to capping distrust for
    // closer tags)
    // double distanceFactor =
    // baseNoise + distanceExponentialCoefficient * Math.pow(distanceExponentialBase, distance);

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

    DogLog.log("Vision/calibrationFactor", calibrationFactor);
    DogLog.log("Vision/tagFactor", tagFactor);
    DogLog.log("Vision/distanceFactor", distanceFactor);
    DogLog.log("Vision/speedFactor", speedFactor);

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
