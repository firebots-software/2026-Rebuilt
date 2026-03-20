package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeVision extends SubsystemBase {

  private final PhotonCamera photonCamera;
  private PhotonPipelineResult latestVisionResult;

  public IntakeVision(Constants.FuelGaugeDetection.FuelGaugeCamera cameraID) {
    photonCamera = new PhotonCamera(cameraID.toString());
  }

  @Override
  public void periodic() {}
}
