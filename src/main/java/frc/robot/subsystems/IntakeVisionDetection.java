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
    DogLog.log("Subsystems/FuelGauge/CameraStatus", cameraConnected);
    return cameraConnected;
  }

  private boolean validVisionResult(List<PhotonPipelineResult> results) {
    if (results.isEmpty()) return false;

    latestVisionResult = results.get(results.size() - 1);

    return (latestVisionResult == null);
  }

  private void updateVisionResult() {
    Optional<PhotonTrackedTarget> target = getLargestTarget();
    target.ifPresentOrElse(
        t -> {
          latestRawArea = t.getArea();
          latestRawYaw = t.getYaw();
        },
        () -> DogLog.log("Subsystems/IntakeVision/TargetPresent", false));
  }

  private Optional<PhotonTrackedTarget> getLargestTarget() {
    if (latestVisionResult == null) return Optional.empty();
    List<PhotonTrackedTarget> targets = latestVisionResult.getTargets();
    if (targets.isEmpty()) return Optional.empty();

    return targets.stream().max((a, b) -> Double.compare(a.getArea(), b.getArea()));
  }

  public double getYaw() {
    return latestRawYaw;
  }
}
