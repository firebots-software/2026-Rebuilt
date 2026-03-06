package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Vision.VisionCamera;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {

  private final Constants.Vision.VisionCamera cameraID;

  private String cameraTitle;
  private String loggingPath;

  // normalization maximums
  private double maximumRobotSpeed = Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;

  // references for PhotonVision
  private final PhotonCamera photonCamera;
  private final PhotonPoseEstimator poseEstimator;
  private PhotonPipelineResult latestVisionResult;
  Optional<EstimatedRobotPose> visionEst;
  private final AprilTagFieldLayout fieldLayout;

  // addFilteredPose() vals
  private boolean hasValidMeasurement;
  Pose2d latestMeasuredPose;
  double latestFinalTimestamp;
  Matrix<N3, N1> latestNoiseVector;

  // constructor for VisionSubsystem
  public VisionSubsystem(Constants.Vision.VisionCamera cameraID) {

    this.cameraID = cameraID;
    photonCamera = new PhotonCamera(cameraID.toString());
    Transform3d robotToCamera = cameraID.getCameraTransform();

    this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    poseEstimator = new PhotonPoseEstimator(fieldLayout, robotToCamera);
    poseEstimator.setFieldTags(fieldLayout);

    cameraTitle = cameraID.getLoggingName();
    loggingPath = "Subsystems/Vision/" + cameraTitle;
    latestVisionResult = null;
  }

  public VisionCamera getCamera() {
    return cameraID;
  }

  @Override
  public void periodic() {

    setupPeriodicVars();

    List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();
    for (PhotonPipelineResult result : results) {
      latestVisionResult = result;
      visionEst = poseEstimator.estimateCoprocMultiTagPose(result);
      if (visionEst.isEmpty()) visionEst = poseEstimator.estimateLowestAmbiguityPose(result);
    }

    DogLog.log(loggingPath + "/CameraConnected", true);
  }

  private void setupPeriodicVars() {
    visionEst = Optional.empty();
    latestVisionResult = null;
    hasValidMeasurement = false;
  }

  public void calculateFilteredPose(CommandSwerveDrivetrain swerve) {
    hasValidMeasurement = false;

    if (!checkResultValidity()) return;

    // creates a list of all detected tags and logs for debugging
    List<PhotonTrackedTarget> tags =
        latestVisionResult.getTargets().stream().collect(Collectors.toList());

    // Get detected tags
    tags = latestVisionResult.getTargets();
    if (tags.isEmpty()) {
      DogLog.log(loggingPath + "/Tags", false);
      return;
    }
    DogLog.log(loggingPath + "/Tags", true);

    // log area and yaw for all detected april tags
    for (PhotonTrackedTarget tag : tags) {
      DogLog.log(loggingPath + "/Tags/" + tag.getFiducialId() + "/Area", tag.getArea());
      DogLog.log(loggingPath + "/Tags/" + tag.getFiducialId() + "/Yaw", tag.getYaw());
    }

    EstimatedRobotPose estimatedPose = visionEst.get();
    Pose2d measuredPose = estimatedPose.estimatedPose.toPose2d();
    DogLog.log(loggingPath + "/MeasuredPose", measuredPose);

    double minDistance = getMinDistance();

    double averageDistance = getAverageDistance();

    if (Double.isNaN(minDistance) || minDistance > Constants.Vision.MAX_TAG_DISTANCE) {
      DogLog.log(loggingPath + "/ThrownOutDistance", true);
      return;
    }
    DogLog.log(loggingPath + "/ThrownOutDistance", false);

    // Log yaw + area for debugging
    for (PhotonTrackedTarget tag : tags) {
      DogLog.log(loggingPath + "/TagYaw", tag.getYaw());
      DogLog.log(loggingPath + "/TagArea", tag.getArea());
    }

    int tagCount = tags.size();
    DogLog.log("Subsystems/Vision/tagCount", tagCount);

    ChassisSpeeds robotSpeeds = swerve.getState().Speeds;

    var field =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, swerve.getState().Pose.getRotation());

    double currentSpeed = Math.hypot(field.vxMetersPerSecond, field.vyMetersPerSecond);

    // Compute noise model
    double nX =
        computeNoiseXY(
            Constants.Vision.BASE_NOISE_X,
            Constants.Vision.DISTANCE_EXPONENTIAL_COEFFICIENT_X,
            Constants.Vision.DISTANCE_EXPONENTIAL_BASE_X,
            Constants.Vision.ANGLE_COEFFICIENT_X,
            Constants.Vision.SPEED_COEFFICIENT_X,
            averageDistance,
            currentSpeed,
            tagCount);

    double nY =
        computeNoiseXY(
            Constants.Vision.BASE_NOISE_Y,
            Constants.Vision.DISTANCE_EXPONENTIAL_COEFFICIENT_Y,
            Constants.Vision.DISTANCE_EXPONENTIAL_BASE_Y,
            Constants.Vision.ANGLE_COEFFICIENT_Y,
            Constants.Vision.SPEED_COEFFICIENT_Y,
            averageDistance,
            currentSpeed,
            tagCount);

    double nTH =
        computeNoiseHeading(
            Constants.Vision.BASE_NOISE_THETA,
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

    hasValidMeasurement = true;

    if (measuredPose == null) {
      DogLog.log("Subsystems/Vision/measuredPoseAvailable", false);
    } else {
      DogLog.log("Subsystems/Vision/measuredPoseAvailable", true);
    }
  }

  private boolean checkResultValidity() {
    boolean validResult = true;
    DogLog.log(loggingPath + "/addFilteredPoseRunning", true);

    if (latestVisionResult == null || latestVisionResult.getTargets().isEmpty()) {
      DogLog.log(loggingPath + "/HasResultsTargets", false);
      validResult = false;
    } else {
      DogLog.log(loggingPath + "/HasResultsTargets", true);
    }

    if (visionEst.isEmpty()) {
      DogLog.log(loggingPath + "/HasEstimate", false);
      validResult = false;
    } else {
      DogLog.log(loggingPath + "/HasEstimate", true);
    }

    return validResult;
  }

  public double getMinDistance() {
    double minDist =
        (latestVisionResult == null || latestVisionResult.getTargets().isEmpty())
            ? Double.MAX_VALUE
            : latestVisionResult.getTargets().stream()
                .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                .min()
                .orElse(Double.NaN);

    DogLog.log(loggingPath + "/ClosestTagDistance", minDist);
    return minDist;
  }

  public double getAverageDistance() {
    double avgDist =
        (latestVisionResult == null || latestVisionResult.getTargets().isEmpty())
            ? Double.MAX_VALUE
            : latestVisionResult.getTargets().stream()
                .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                .average()
                .orElse(Double.NaN);

    DogLog.log(loggingPath + "/AverageTagDistance", avgDist);
    return avgDist;
  }

  public double getMaxDistance() {
    double maxDist =
        (latestVisionResult == null || latestVisionResult.getTargets().isEmpty())
            ? Double.MAX_VALUE
            : latestVisionResult.getTargets().stream()
                .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                .max()
                .orElse(Double.NaN);

    DogLog.log(loggingPath + "/MaxTagDistance", maxDist);
    return maxDist;
  }

  public double getPoseAmbiguity() {
    double poseAmbiguity =
        (latestVisionResult == null || latestVisionResult.getTargets().isEmpty())
            ? Double.MAX_VALUE
            : latestVisionResult.getTargets().stream()
                .mapToDouble(PhotonTrackedTarget::getPoseAmbiguity)
                .average()
                .orElse(Double.NaN);

    DogLog.log(loggingPath + "/PoseAmbiguity", poseAmbiguity);
    return poseAmbiguity;
  }

  public boolean hasValidMeasurement() {
    return hasValidMeasurement;
  }

  private void processPoseEstimate(
      Pose2d measuredPose,
      double averageDistance,
      double currentSpeed,
      int tagCount,
      double timestamp,
      Matrix<N3, N1> noiseVector) {

    // Use vision timestamp if within threshold of FPGA timestamp, else take the FPGA timestamp with
    // correction.
    double fpgaTimestamp = Timer.getFPGATimestamp();
    double timestampDiff = Math.abs(timestamp - fpgaTimestamp);
    double finalTimestamp =
        (timestampDiff > Constants.Vision.TIMESTAMP_THRESHOLD)
            ? fpgaTimestamp + Constants.Vision.TIMESTAMP_FPGA_CORRECTION
            : timestamp;

    latestMeasuredPose = measuredPose;
    latestFinalTimestamp = finalTimestamp;
    latestNoiseVector = noiseVector;
  }

  public Pose2d getFilteredPose() {
    return latestMeasuredPose;
  }

  public void addFilteredPose(CommandSwerveDrivetrain swerve) {
    swerve.addVisionMeasurement(latestMeasuredPose, latestFinalTimestamp, latestNoiseVector);
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

    double computedStdDevs =
        Constants.Vision.CALIBRATION_FACTOR * tagFactor * distanceFactor * speedFactor;
    return computedStdDevs;
  }
}
