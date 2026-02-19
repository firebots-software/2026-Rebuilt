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

  // normalization maximums
  private double maximumRobotSpeed = Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;

  // Noise parameters
  private double calibrationFactor = 1d; // constant multiplier to everything
  private double baseNoiseX = 0.0008; // meters
  private double baseNoiseY = 0.0008;
  private double baseNoiseTheta = 0.5; // radians

  // references for PhotonVision
  private final PhotonCamera photonCamera;
  private final PhotonPoseEstimator poseEstimator;
  private PhotonPipelineResult latestVisionResult;
  Optional<EstimatedRobotPose> visionEst;
  private final AprilTagFieldLayout fieldLayout;
  private boolean hasValidMeasurement;

  public static final double timestampDiffThreshold = 0.5;
  public static final double timestampFPGACorrection = -0.03;

  // addFilteredPose() vals
  Pose2d latestMeasuredPose;
  double latestFinalTimestamp;
  Matrix<N3, N1> latestNoiseVector;

  // constructor for VisionSubsystem
  public VisionSubsystem(Constants.Vision.VisionCamera cameraID) {

    this.cameraID = cameraID;
    photonCamera = new PhotonCamera(cameraID.toString());
    Transform3d robotToCamera = cameraID.getCameraTransform();

    // load field layout
    this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // initialize poseEstimator
    poseEstimator = new PhotonPoseEstimator(fieldLayout, robotToCamera);

    cameraTitle = cameraID.getLoggingName();
    latestVisionResult = null;
  }

  @Override
  public void periodic() {

    visionEst = Optional.empty();
    latestVisionResult = null;
    hasValidMeasurement = false;

    List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();
    for (PhotonPipelineResult result : results) {
      latestVisionResult = result;
      visionEst = poseEstimator.estimateCoprocMultiTagPose(result);
      if (visionEst.isEmpty()) visionEst = poseEstimator.estimateLowestAmbiguityPose(result);
    }

    DogLog.log("Subsystems/Vision/" + cameraTitle + "/CameraConnected", true);
  }

  public VisionCamera getCamera() {
    return cameraID;
  }

  public void calculateFilteredPose(CommandSwerveDrivetrain swerve) {
    hasValidMeasurement = false;

    DogLog.log("Subsystems/Vision/addFilteredPoseworking", true);

    if (latestVisionResult == null || latestVisionResult.getTargets().isEmpty()) {
      DogLog.log("Subsystems/Vision/" + cameraTitle + "/HasResult", false);
      return;
    }
    DogLog.log("Subsystems/Vision/" + cameraTitle + "/HasResult", true);

    // Ensure we have a valid pose estimate and vision result from periodic()
    if (visionEst.isEmpty()) {
      DogLog.log("Subsystems/Vision/" + cameraTitle + "/HasEstimate", false);
      return;
    }
    DogLog.log("Subsystems/Vision/" + cameraTitle + "/HasEstimate", true);

    // creates a list of all detected tags and logs for debugging
    List<PhotonTrackedTarget> tags =
        latestVisionResult.getTargets().stream().collect(Collectors.toList());

    // Get detected tags
    tags = latestVisionResult.getTargets();
    if (tags.isEmpty()) {
      DogLog.log("Subsystems/Vision/" + cameraTitle + "/Tags", false);
      return;
    }
    DogLog.log("Subsystems/Vision/" + cameraTitle + "/Tags", true);

    // log area and yaw for all detected april tags
    for (int tag = 0; tag < tags.size(); tag++) {
      DogLog.log(
          "Subsystems/Vision/" + cameraTitle + "/Tags/" + tag + "/Area", tags.get(tag).getArea());
      DogLog.log(
          "Subsystems/Vision/" + cameraTitle + "/Tags/" + tag + "/Yaw", tags.get(tag).getYaw());
    }

    // Extract pose estimate
    EstimatedRobotPose estimatedPose = visionEst.get();
    Pose2d measuredPose = estimatedPose.estimatedPose.toPose2d();
    DogLog.log("Subsystems/Vision/" + cameraTitle + "/MeasuredPose", measuredPose);

    // distance to closest april tag
    double minDistance = getMinDistance();

    // average distance to all visible april tags
    double averageDistance = getAverageDistance();

    // Reject invalid or distant measurements
    if (Double.isNaN(minDistance) || minDistance > Constants.Vision.MAX_TAG_DISTANCE) {
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

    ChassisSpeeds robotSpeeds = swerve.getState().Speeds;

    var field =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, swerve.getState().Pose.getRotation());

    double currentSpeed = Math.hypot(field.vxMetersPerSecond, field.vyMetersPerSecond);

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

    hasValidMeasurement = true;

    if (measuredPose == null) {
      DogLog.log("Subsystems/Vision/measuredPoseAvailable", false);
    } else {
      DogLog.log("Subsystems/Vision/measuredPoseAvailable", true);
    }
  }

  public double getMinDistance() {
    double minDist =
        (latestVisionResult == null || latestVisionResult.getTargets().isEmpty())
            ? Double.MAX_VALUE
            : latestVisionResult.getTargets().stream()
                .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                .min()
                .orElse(Double.NaN);

    DogLog.log("Subsystems/Vision/" + cameraTitle + "/ClosestTagDistance", minDist);
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

    DogLog.log("Subsystems/Vision/" + cameraTitle + "/AverageTagDistance", avgDist);
    return avgDist;
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

    // Use vision timestamp if within threshold of FPGA timestamp
    double fpgaTimestamp = Timer.getFPGATimestamp();
    double timestampDiff = Math.abs(timestamp - fpgaTimestamp);
    double finalTimestamp =
        (timestampDiff > timestampDiffThreshold)
            ? fpgaTimestamp + timestampFPGACorrection
            : timestamp;

    latestMeasuredPose = measuredPose;
    latestFinalTimestamp = finalTimestamp;
    latestNoiseVector = noiseVector;
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
