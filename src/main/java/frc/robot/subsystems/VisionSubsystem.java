package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Vision.VisionCamera;
import frc.robot.util.VisionUtils;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
  private final Constants.Vision.VisionCamera cameraID;
  private final PhotonCamera photonCamera;
  private final AprilTagFieldLayout fieldLayout;
  private final PhotonPoseEstimator poseEstimator;
  private PhotonPipelineResult latestVisionResult;

  private String cameraTitle;
  private String loggingPath;

  private Optional<EstimatedRobotPose> visionEstimate;

  private boolean cameraConnectedStatus = false;

  private Matrix<N3, N3> cameraIntrinsics;
  private Matrix<N8,N1> distortionCoeffs;

  // addFilteredPose() vals
  private boolean hasValidMeasurement;
  private Pose2d latestMeasuredPose;
  private double latestFinalTimestamp;
  private Matrix<N3, N1> latestNoiseVector;
  private double latestMinDistance;
  private double latestMaxDistance;
  private double latestAvgDistance;
  private int latestTagCount;
  private CommandSwerveDrivetrain swerve;
  private Transform3d camHeight;

  public VisionSubsystem(
      Constants.Vision.VisionCamera cameraID, CommandSwerveDrivetrain drivetrain) {
    this.cameraID = cameraID;
    photonCamera = new PhotonCamera(cameraID.toString());
    Transform3d robotToCamera = cameraID.getCameraTransform();
    camHeight = new Transform3d(0.0, 0.0, robotToCamera.getZ(), new Rotation3d(0.0, 0.0, 0.0));

    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    poseEstimator = new PhotonPoseEstimator(fieldLayout, robotToCamera);
    latestVisionResult = null;

    cameraTitle = cameraID.getLoggingName();
    loggingPath = "Subsystems/Vision/" + cameraTitle;

    this.swerve = drivetrain;

    cameraIntrinsics = cameraID.getCameraMatrix();
    distortionCoeffs = cameraID.getDistCoeffs();
  }

  public VisionCamera getCameraID() {
    return cameraID;
  }

  @Override
  public void periodic() {
    visionEstimate = Optional.empty();
    latestVisionResult = null;
    hasValidMeasurement = false;

    if (!cameraConnected()) return;

    updateEstimate(photonCamera.getAllUnreadResults());
  }

  private boolean cameraConnected() {
    cameraConnectedStatus = photonCamera.isConnected();
    DogLog.log(loggingPath + "/CameraConnected", cameraConnectedStatus);
    return cameraConnectedStatus;
  }

  private void updateEstimate(List<PhotonPipelineResult> results) {
    if (results.isEmpty()) return;
    latestVisionResult = results.get(results.size() - 1);

    visionEstimate = poseEstimator.estimateCoprocMultiTagPose(latestVisionResult);
    if (visionEstimate.isEmpty())
      visionEstimate =
          Constants.pnpReady
              ? poseEstimator.estimateConstrainedSolvepnpPose(
                  latestVisionResult,
                  cameraIntrinsics,
                  distortionCoeffs,
                  new Pose3d(swerve.getCurrentState().Pose).plus(camHeight),
                  false,
                  Constants.Vision.HDG_PENALTY)
              : poseEstimator.estimateClosestToReferencePose(
                  latestVisionResult, new Pose3d(swerve.getCurrentState().Pose).plus(camHeight));
  }

  public void calculateFilteredPose() {
    hasValidMeasurement = false;

    if (!resultValid()) return;
    if (!tagsValid()) return;

    EstimatedRobotPose estimatedPose = visionEstimate.get();
    latestMeasuredPose = estimatedPose.estimatedPose.toPose2d();
    DogLog.log(loggingPath + "/MeasuredPose", latestMeasuredPose);

    latestMinDistance = getMinDistance();
    //latestMaxDistance = getMaxDistance();
    latestAvgDistance = getAverageDistance();

    if (throwOutDistance(latestMinDistance)) return;

    throwOutHeadingChange(latestMeasuredPose);

    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            swerve.getState().Speeds, swerve.getState().Pose.getRotation());

    double currentSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

    latestNoiseVector =
        VisionUtils.computeNoiseVector(latestAvgDistance, currentSpeed, latestTagCount);
    latestFinalTimestamp = calculateTimestamp(estimatedPose.timestampSeconds);
    hasValidMeasurement = true;

    DogLog.log(loggingPath + "/measuredPoseAvailable", latestMeasuredPose == null);
  }

  private boolean resultValid() {
    DogLog.log(loggingPath + "/addFilteredPoseRunning", true);

    boolean resultExists = latestVisionResult != null && !latestVisionResult.getTargets().isEmpty();
    DogLog.log(loggingPath + "/HasResultsTargets", resultExists);

    boolean hasEstimate = !visionEstimate.isEmpty();
    DogLog.log(loggingPath + "/HasEstimate", hasEstimate);

    return resultExists && hasEstimate;
  }

  private boolean tagsValid() {
    List<PhotonTrackedTarget> tags = latestVisionResult.getTargets();

    boolean tagsValid = !tags.isEmpty();
    DogLog.log(loggingPath + "/Tags", tagsValid);

    if (!tagsValid) return false;

    for (PhotonTrackedTarget tag : tags) {
      DogLog.log(loggingPath + "/Tags/" + tag.getFiducialId() + "/Area", tag.getArea());
      DogLog.log(loggingPath + "/Tags/" + tag.getFiducialId() + "/Yaw", tag.getYaw());
    }
    latestTagCount = tags.size();
    DogLog.log("Subsystems/Vision/tagCount", latestTagCount);

    return true;
  }

  private boolean throwOutDistance(double min) {
    boolean thrownOut = Double.isNaN(min) || min > Constants.Vision.MAX_TAG_DISTANCE;
    DogLog.log(loggingPath + "/ThrownOutDistance", thrownOut);
    return thrownOut;
  }

  private void throwOutHeadingChange(Pose2d pose) {
    Rotation2d estimatedHeading = pose.getRotation();
    Rotation2d currentHeading = swerve.getCurrentState().Pose.getRotation();
    double rotationDiff = Math.abs(estimatedHeading.relativeTo(currentHeading).getDegrees());
    boolean trueIfThrown = rotationDiff > VisionUtils.getHeadingThreshold();
    DogLog.log(loggingPath + "/ThrownOutHeading", trueIfThrown);
    DogLog.log(loggingPath + "/ThrownOutHeadingDiff", rotationDiff);
  }

  public boolean getCameraConnected() {
    return cameraConnectedStatus;
  }

  public double getMinDistance() {
    double minDist =
        VisionUtils.visionResultInvalid(latestVisionResult)
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
        VisionUtils.visionResultInvalid(latestVisionResult)
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
        VisionUtils.visionResultInvalid(latestVisionResult)
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
        VisionUtils.visionResultInvalid(latestVisionResult)
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

  private double calculateTimestamp(double timestamp) {
    double fpgaTimestamp = Timer.getFPGATimestamp();
    double timestampDiff = Math.abs(timestamp - fpgaTimestamp);
    double finalTimestamp =
        timestampDiff > Constants.Vision.TIMESTAMP_THRESHOLD
            ? fpgaTimestamp + Constants.Vision.TIMESTAMP_FPGA_CORRECTION
            : timestamp;

    return finalTimestamp;
  }

  public Pose2d getFilteredPose() {
    return latestMeasuredPose;
  }

  public void addFilteredPose() {
    swerve.addVisionMeasurement(latestMeasuredPose, latestFinalTimestamp, latestNoiseVector);
  }
}
