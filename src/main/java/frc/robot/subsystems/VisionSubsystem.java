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
import java.util.stream.Collectors;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
  // static member that contains array of all VisionSubsytem cameras
  private static VisionSubsystem[] cameraList =
      new VisionSubsystem[Constants.Vision.Cameras.values().length];

  // Camera datatype with only 3 options, left, right, or color
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

    // initialize poseEstimator
    if (cameraID != Constants.Vision.Cameras.COLOR_CAM) {
      poseEstimator = new PhotonPoseEstimator(fieldLayout, cameraToRobot);
    } else {
      poseEstimator = null; // color camera does not need pose estimation
    }

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
    DogLog.log("Vision/isRunning", true);
    // confirm that all cameras are connected and update cameraConnected respectively
    boolean cameraConnected = photonCamera.isConnected();
    if (!cameraConnected) {
      DogLog.log("Vision/" + cameraTitle + "/CameraConnected", false);
      return;
    }

    DogLog.log("Vision/" + cameraTitle + "/CameraConnected", true);

    // add all unread results to results <List>
    List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();

    // Go through all results (if there are any) and update the latest result with the last
    for (var result : results) latestVisionResult = result;

    // log yaw and area of blob if present
    Optional<PhotonTrackedTarget> blob = getLargestBlob();
    blob.ifPresentOrElse(
        b -> {
          DogLog.log("Vision/BlobPresent", true);
          DogLog.log("Vision/BlobYaw", b.getYaw());
          DogLog.log("Vision/BlobPitch", b.getPitch());
          DogLog.log("Vision/BlobSkew", b.getSkew());
        },
        () -> DogLog.log("Vision/BlobPresent", false));
  }

  public void addFilteredPose() {
    // skip pose estimation for color camera
    if (poseEstimator == null) return;

    // initialize new poseEstimator
    PhotonPoseEstimator estimator = poseEstimator;

    // check if camera has targets visible and log
    if (latestVisionResult == null || latestVisionResult.getTargets().isEmpty()) {
      DogLog.log("Vision/" + cameraTitle + "/HasTargets", false);
      return;
    }
    DogLog.log("Vision/" + cameraTitle + "/HasTargets", true);

    // distance to closest april tag
    double minDistance =
        latestVisionResult.getTargets().stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .min()
            .orElse(Double.NaN);

    DogLog.log("Vision/closestTagDistance", minDistance);

    // average distance to all visible april tags
    double averageDistance =
        latestVisionResult.getTargets().stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(Double.NaN);
    DogLog.log("Vision/averageTagDistance", averageDistance);

    // 2025-reefscape has a validTags list on lines 160-166, replacing it with a list of all tags
    // for 26

    // skip AprilTag pose estimation for color cameras
    if (cameraID == Constants.Vision.Cameras.COLOR_CAM) {
      getLargestBlob()
          .ifPresent(
              blob -> {
                DogLog.log("Vision/BlobPresent", true);
                DogLog.log("Vision/BlobYaw", blob.getYaw());
                DogLog.log("Vision/BlobArea", blob.getArea());
                // calculating fuel gage percentage by dividing area of the ball by the max ball
                // area then multiplying by 100 and rounding to nearest 10
                DogLog.log(
                    "Vision/FuelGuage",
                    ((double)
                            Math.round(
                                blob.getArea()
                                    / Constants.Vision.MAX_DETECTABLE_FUEL_AREA_PERCENTAGE
                                    * 100.0
                                    / 10.0))
                        * 10.0);
                DogLog.log(
                    "Vision/FuelGuageRealistic",
                    ((double)
                            Math.round(
                                blob.getArea()
                                    / Constants.Vision.REALISTIC_MAX_DETECTABLE_AREA_PERCENTAGE
                                    * 100.0
                                    / 10.0))
                        * 10.0);
              });
      return;
    }

    // creates a list of all detected tags and logs for debugging
    List<PhotonTrackedTarget> tags =
        latestVisionResult.getTargets().stream().collect(Collectors.toList());

    // log area and yaw for all detected april tags
    for (PhotonTrackedTarget tag : tags) {
      DogLog.log("Vision/" + cameraTitle + "/Area", tag.getArea());
      DogLog.log("Vision/" + cameraTitle + "/Yaw", tag.getYaw());
    }

    if (tags.isEmpty()) {
      DogLog.log("Vision/" + cameraTitle + "/Tags", false);
      return;
    }
    DogLog.log("Vision/" + cameraTitle + "/Tags", true);

    // do nothing if no minDistance detected or if minDistance is too large
    if (Double.isNaN(minDistance) || minDistance > maxDistance) {
      DogLog.log("Vision/" + cameraTitle + "/ThrownOutDistance", true);
      return;
    }
    DogLog.log("Vision/" + cameraTitle + "/ThrownOutDistance", false);

    // TODO: re-implement lines 200-205 in 2025 repo

    int tagCount = tags.size();

    // get from swerve
    double currentSpeed = 0d;

    // check for and get pose from PhotonVision
    Optional<EstimatedRobotPose> poseEstimationUpdate =
        estimator.estimateCoprocMultiTagPose(latestVisionResult);
    if (poseEstimationUpdate.isEmpty()) {
      DogLog.log("Vision/" + cameraTitle + "/AvailablePose", false);
      return;
    }
    DogLog.log("Vision/" + cameraTitle + "/AvailablePose", true);

    // get estimatedPose and convert it to Pose2d
    EstimatedRobotPose estimatedPose = poseEstimationUpdate.get();
    Pose2d measuredPose = estimatedPose.estimatedPose.toPose2d();
    DogLog.log("Vision/" + cameraTitle + "/Pose2d", measuredPose);

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

    processPoseEstimate(
        measuredPose,
        averageDistance,
        currentSpeed,
        tagCount,
        latestVisionResult.getTimestampSeconds(),
        noiseVector);

    if (poseEstimationUpdate.isPresent()) {
      EstimatedRobotPose estimatedRobotPose = poseEstimationUpdate.get();
      Pose2d visionPose = estimatedRobotPose.estimatedPose.toPose2d();
      DogLog.log("Vision/PoseEstimationAvailable", true);

      DogLog.log("Vision/VisionPoseEstimate", visionPose);
    } else {
      DogLog.log("Vision/PoseEstimationAvailable", false);
    }

    // TODO: re-implement lines 218-281 in 2025 repo
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
    double distanceFactor =
        baseNoise + distanceExponentialCoefficient * Math.pow(distanceExponentialBase, distance);

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

  // object detection

  public Optional<PhotonTrackedTarget> getLargestBlob() {
    if (latestVisionResult == null) return Optional.empty();
    List<PhotonTrackedTarget> targets = latestVisionResult.getTargets();
    if (targets.isEmpty()) return Optional.empty();

    // returns the largest by area
    return targets.stream().max((a, b) -> Double.compare(a.getArea(), b.getArea()));
  }

  public Optional<Double> getYawToBlob() {
    return getLargestBlob().map(PhotonTrackedTarget::getYaw);
  }

  public Optional<Double> getAreaOfBlob() {
    return getLargestBlob().map(PhotonTrackedTarget::getArea);
  }

  public Optional<Double> getPitchToBlob() {
    return getLargestBlob().map(PhotonTrackedTarget::getPitch);
  }

  public Optional<Double> getSkewToBlob() {
    return getLargestBlob().map(PhotonTrackedTarget::getSkew);
  }
}
