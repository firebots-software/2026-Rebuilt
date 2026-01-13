package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.List;
import java.util.function.BooleanSupplier;
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
    poseEstimator = new PhotonPoseEstimator(
      fieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      cameraToRobot
    );

    cameraTitle = cameraID.getLoggingName();
      latestVisionResult = null;
    }

  // returns VisionSubsystem instance
  public static VisionSubsystem getInstance(Constants.Vision.Cameras cameraID, BooleanSupplier isRedSide) {
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

    //check if camera has targets visible and log
    if (latestVisionResult == null || latestVisionResult.getTargets().isEmpty()) {
      DogLog.log("Vision/" + cameraTitle + "/HasTargets", false);
      return;
    }
    DogLog.log("Vision/" + cameraTitle + "/HasTargets", true);
  }

  // to be completed; method aims to combine final pose estimate with odometry for accurate estimation
  private void processPoseEstimate(
    Pose2d measuredPose,
    double averageDistance,
    double currentSpeed,
    int tagCount,
    double timestamp,
    Matrix<N3, N1> noiseVector) {}
}