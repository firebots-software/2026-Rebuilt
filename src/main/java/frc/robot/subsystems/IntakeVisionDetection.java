package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class IntakeVisionDetection extends SubsystemBase {
  private final PhotonCamera photonCamera;
  private PhotonPipelineResult latestVisionResult;
  private double latestRawArea;
  private double latestRawYaw;
  private double latestRawPitch;

  public IntakeVisionDetection(Constants.IntakeVision.IntakeVisionCamera cameraID) {
    photonCamera = new PhotonCamera(cameraID.toString());
  }

  @Override
  public void periodic() {
    if (!cameraConnected()) return;

    if (!validVisionResult(photonCamera.getAllUnreadResults())) return;

    updateVisionResult();
  }

  private boolean cameraConnected() {
    boolean cameraConnected = photonCamera.isConnected();
    DogLog.log("Subsystems/IntakeVision/CameraStatus", cameraConnected);
    return cameraConnected;
  }

  private boolean validVisionResult(List<PhotonPipelineResult> results) {
    if (results.isEmpty()) return false;

    latestVisionResult = results.get(results.size() - 1);

    DogLog.log("Subsystems/IntakeVision/ResultExists", latestVisionResult != null);

    return latestVisionResult != null;
  }

  private void updateVisionResult() {
    Optional<PhotonTrackedTarget> target = getLargestTarget();
    target.ifPresentOrElse(
        t -> {
          DogLog.log("Subsystems/IntakeVision/TargetPresent", true);
          latestRawArea = t.getArea();
          latestRawYaw = t.getYaw();
          latestRawPitch = t.getPitch();
          DogLog.log("Subsystems/IntakeVision/Area", latestRawArea);
          DogLog.log("Subsystems/IntakeVision/Yaw", latestRawYaw);
          DogLog.log("Subsystems/IntakeVision/Pitch", latestRawPitch);
        },
        () -> DogLog.log("Subsystems/IntakeVision/TargetPresent", false));
  }

  private Optional<PhotonTrackedTarget> getLargestTarget() {
    if (latestVisionResult == null) return Optional.empty();

    List<PhotonTrackedTarget> targets = latestVisionResult.getTargets();
    DogLog.log("Subsystems/IntakeVision/TargetsLength", targets.size());

    if (targets.isEmpty()) return Optional.empty();

    return targets.stream().max((a, b) -> Double.compare(a.getArea(), b.getArea()));
  }

  public double getYaw() {
    DogLog.log("Subsystems/IntakeVision/LatestRetrivedYaw", -latestRawYaw);
    return -latestRawYaw;
  }

  public double getArea() {
    return latestRawArea;
  }

  public double getPitch() {
    DogLog.log("Subsystems/IntakeVision/LatestRetrivedPitch", -latestRawPitch);
    return -latestRawPitch;
  }
}