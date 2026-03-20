package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class IntakeVision extends SubsystemBase {

  private final PhotonCamera photonCamera;
  private PhotonPipelineResult latestVisionResult;

  public IntakeVision(Constants.FuelGaugeDetection.FuelGaugeCamera cameraID) {
    photonCamera = new PhotonCamera(cameraID.toString());
  }

  @Override
  public void periodic() {
    if (!cameraConnected()) return;

    if (!validVisionResult(photonCamera.getAllUnreadResults())) return;
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
}
