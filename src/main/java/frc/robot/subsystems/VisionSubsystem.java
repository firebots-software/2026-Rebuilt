package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
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
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
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






  // NOTE FOR SID/SAKETH: come back to ln 49-75 in 2025 repo

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

  // constructor for VisionSubsystem
  public VisionSubsystem(Constants.Vision.Cameras cameraID, BooleanSupplier isRedSide) {
    this.isRedSide = isRedSide;
    this.cameraID = cameraID;
    photonCamera = new PhotonCamera(cameraID.toString());
    Transform3d cameraToRobot = Constants.Vision.getCameraTransform(cameraID);

    // load field layout
    this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    // initialize poseEstimator
    poseEstimator =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraToRobot);

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
    // confirm that all cameras are connected and update cameraConnected respectively
    boolean cameraConnected = photonCamera.isConnected();
    if (!cameraConnected) {
      DogLog.log("Vision/" + cameraTitle + "/CameraConnected", false);
      return;
    }

    DogLog.log("Vision/" + cameraTitle + "/CameraConnected", true);

    // add all unread results to results <List>
    List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();

    // comb through every result in results
    for (var result : results) latestVisionResult = result;
  }

  public void addFilteredPose() {
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

    // average distance to all visible april tags
    double averageDistance = latestVisionResult.getTargets().stream()
      .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
      .average()
      .orElse(Double.NaN);

    // 2025-reefscape has a validTags list on lines 160-166, replacing it with a list of all tags for 26

    // creates a list of all detected tags and logs for debugging
    List<PhotonTrackedTarget> tags = latestVisionResult.getTargets().stream()
      .collect(Collectors.toList());
    
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

    // check for and get pose from PhotonVision
    Optional<EstimatedRobotPose> maybePose = estimator.update(latestVisionResult);
    if (maybePose.isEmpty()) {
      DogLog.log("Vision/" + cameraTitle + "/AvailablePose", false);
      return;
    }
    DogLog.log("Vision/" + cameraTitle + "/AvailablePose", true);

    // get estimatedPose and convert it to Pose2d
    EstimatedRobotPose estimatedPose = maybePose.get();
    Pose2d measuredPose = estimatedPose.estimatedPose.toPose2d();

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
        (timestampDiff > Constants.Vision.TIMESTAMP_DIFF_THRESHOLD)
            ? fpgaTimestamp + Constants.Vision.TIMESTAMP_FPGA_CORRECTION
            : timestamp;

    // swerveDrive.addVisionMeasurement(measuredPose...) check ln 279
  }

  // prev. year had isTagOnActiveSide but not relevant this year

  private boolean yawAcceptable(double yaw) {
    boolean acceptable = Math.abs(yaw) < Constants.Vision.ACCEPTABLE_YAW;
    DogLog.log("Vision/acceptableYaw", acceptable);

    return acceptable;
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
    int effectiveTags = Math.min(tagCount,  4);
    double tagFactor = 1d / Math.sqrt(effectiveTags);
    
    // distance term (d^2)
    // if distance less than field size cap the distrust, otherwise don't
    double distanceFactor = (distance < (17.548 + 0.67))
        ? Math.min(baseNoise + distanceExponentialCoefficient * Math.pow(distanceExponentialBase, distance),
            1.167)
        : (baseNoise + distanceExponentialCoefficient * Math.pow(distanceExponentialBase, distance));
    
    // Speed term (quadratic)
    double vNorm = Math.min(robotSpeed, maximumRobotSpeed) / maximumRobotSpeed;
    double speedFactor = 1d + speedCoefficient * (vNorm*vNorm);
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
    int effectiveTags = Math.min(tagCount,  4);
    double tagFactor = 1d / Math.sqrt(effectiveTags);

    // Distance term (d^2)
    double distanceFactor = baseNoise + distanceCoefficient * distance * distance;

    double vNorm = Math.min(robotSpeed, maximumRobotSpeed) / maximumRobotSpeed;
    double speedFactor = 1d + speedCoefficient * (vNorm * vNorm);

    double computedStdDevs = calibrationFactor * tagFactor * distanceFactor * speedFactor;
    return computedStdDevs;
  }


}