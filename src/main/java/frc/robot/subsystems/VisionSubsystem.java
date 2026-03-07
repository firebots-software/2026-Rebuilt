package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Vision.VisionCamera;
import frc.robot.util.VisionUtils;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {

  private final Constants.Vision.VisionCamera cameraID;

  private String cameraTitle;
  private String loggingPath;

  // references for PhotonVision
  private final PhotonCamera photonCamera;
  private final PhotonPoseEstimator poseEstimator;
  private PhotonPipelineResult latestVisionResult;
  Optional<EstimatedRobotPose> visionEst;
  private final AprilTagFieldLayout fieldLayout;

  // addFilteredPose() vals
  private boolean hasValidMeasurement;
  Pose2d latestMeasuredPose;
  Pose2d previousPose;
  ArrayList<Double> latestJitterMeasurements;
  double latestFinalTimestamp;
  Matrix<N3, N1> latestNoiseVector;
  double latestMinDistance;
  double latestAvgDistance;
  int latestTagCount;

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

    updateVisionEstFromResults(results);

    DogLog.log(loggingPath + "/CameraConnected", true);
  }

  private void setupPeriodicVars() {

    visionEst = Optional.empty();
    latestVisionResult = null;
    hasValidMeasurement = false;
  }

  private void updateVisionEstFromResults(List<PhotonPipelineResult> results) {

    for (PhotonPipelineResult result : results) {
      latestVisionResult = result;
      visionEst = poseEstimator.estimateCoprocMultiTagPose(result);
      if (visionEst.isEmpty()) visionEst = poseEstimator.estimateLowestAmbiguityPose(result);
    }
  }

  public void calculateFilteredPose(CommandSwerveDrivetrain swerve) {

    hasValidMeasurement = false;

    if (!checkResultValidity()) return;

    if (!checkTagsValidity()) return;

    EstimatedRobotPose estimatedPose = visionEst.get();
    Pose2d measuredPose = estimatedPose.estimatedPose.toPose2d();
    DogLog.log(loggingPath + "/MeasuredPose", measuredPose);

    latestMinDistance = getMinDistance();
    latestAvgDistance = getAverageDistance();

    if (throwOutDistance(latestMinDistance)) return;

    ChassisSpeeds robotSpeeds = swerve.getState().Speeds;

    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, swerve.getState().Pose.getRotation());

    double currentSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

    Matrix<N3, N1> noiseVector =
        VisionUtils.computeNoiseVector(latestAvgDistance, currentSpeed, latestTagCount);

    latestFinalTimestamp =
        calculateTimestamp(measuredPose, estimatedPose.timestampSeconds, noiseVector);

    hasValidMeasurement = true;

    DogLog.log(loggingPath + "/measuredPoseAvailable", measuredPose == null);
  }

  private boolean checkResultValidity() {

    DogLog.log(loggingPath + "/addFilteredPoseRunning", true);

    boolean resultExists =
        (!(latestVisionResult == null) && !(latestVisionResult.getTargets().isEmpty()));
    DogLog.log(loggingPath + "/HasResultsTargets", resultExists);

    boolean hasEstimate = !visionEst.isEmpty();
    DogLog.log(loggingPath + "/HasEstimate", hasEstimate);

    return (resultExists && hasEstimate);
  }

  private boolean checkTagsValidity() {

    List<PhotonTrackedTarget> tags = latestVisionResult.getTargets();

    boolean tagsValid = !tags.isEmpty();
    DogLog.log(loggingPath + "/Tags", tagsValid);

    if (tagsValid) {
      for (PhotonTrackedTarget tag : tags) {
        DogLog.log(loggingPath + "/Tags/" + tag.getFiducialId() + "/Area", tag.getArea());
        DogLog.log(loggingPath + "/Tags/" + tag.getFiducialId() + "/Yaw", tag.getYaw());
      }
      latestTagCount = tags.size();
      DogLog.log("Subsystems/Vision/tagCount", latestTagCount);
    }

    return tagsValid;
  }

  private boolean throwOutDistance(double min) {

    boolean thrownOut = (Double.isNaN(min) || min > Constants.Vision.MAX_TAG_DISTANCE);
    DogLog.log(loggingPath + "/ThrownOutDistance", thrownOut);
    return thrownOut;
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

  // Experimental, do not use
  public double getJitter() {
    if (latestMeasuredPose == null || previousPose == null) return Double.MAX_VALUE;
    latestJitterMeasurements.add(
        Math.hypot(
            latestMeasuredPose.getX() - previousPose.getX(),
            latestMeasuredPose.getY() - previousPose.getY()));
    if (latestJitterMeasurements.size() > Constants.Vision.MAX_JITTER_MEASUREMENTS) {
      latestJitterMeasurements.remove(0);
    }
    Double sum = 0.0;
    for (Double j : latestJitterMeasurements) sum += j;
    return sum;
  }

  public boolean hasValidMeasurement() {

    return hasValidMeasurement;
  }

  private double calculateTimestamp(
      Pose2d measuredPose, double timestamp, Matrix<N3, N1> noiseVector) {

    double fpgaTimestamp = Timer.getFPGATimestamp();
    double timestampDiff = Math.abs(timestamp - fpgaTimestamp);
    double finalTimestamp =
        (timestampDiff > Constants.Vision.TIMESTAMP_THRESHOLD)
            ? fpgaTimestamp + Constants.Vision.TIMESTAMP_FPGA_CORRECTION
            : timestamp;

    return finalTimestamp;
  }

  public Pose2d getFilteredPose() {

    return latestMeasuredPose;
  }

  public void addFilteredPose(CommandSwerveDrivetrain swerve) {

    swerve.addVisionMeasurement(latestMeasuredPose, latestFinalTimestamp, latestNoiseVector);
  }
}
