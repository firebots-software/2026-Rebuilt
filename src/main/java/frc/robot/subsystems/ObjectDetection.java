package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ObjectDetection.FuelGauge;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetection extends SubsystemBase {

  private static ObjectDetection[] cameraList =
      new ObjectDetection[Constants.Vision.Cameras.values().length];

  private static ArrayList<Double> latestMeasurements = new ArrayList<>();

  private final Constants.Vision.Cameras cameraID;
  private String cameraTitle;

  private final PhotonCamera photonCamera;
  private PhotonPipelineResult latestVisionResult;

  public ObjectDetection(Constants.Vision.Cameras cameraID) {
    this.cameraID = cameraID;
    photonCamera = new PhotonCamera(cameraID.toString());

    cameraTitle = cameraID.getLoggingName();
  }

  public static ObjectDetection getInstance(Constants.Vision.Cameras cameraID) {
    int index = cameraID.ordinal();
    if (cameraList[index] == null) cameraList[index] = new ObjectDetection(cameraID);
    return cameraList[index];
  }

  public void periodic() {

    List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();
    for (var result : results) latestVisionResult = result;

    Optional<PhotonTrackedTarget> blob = getLargestBlob();
    blob.ifPresentOrElse(
        b -> {
          DogLog.log("Vision/BlobPresent", true);
          DogLog.log("Vision/BlobYaw", b.getYaw());
          DogLog.log("Vision/BlobPitch", b.getPitch());
          DogLog.log("Vision/BlobSkew", b.getSkew());
          double maxFuelPercentage =
              ((double)
                      Math.round(
                          b.getArea()
                              / Constants.Vision.MAX_DETECTABLE_FUEL_AREA_PERCENTAGE
                              * 100.0
                              / 10.0))
                  * 10.0;

          double maxFuelRealisticPercentage =
              ((double)
                      Math.round(
                          b.getArea()
                              / Constants.Vision.REALISTIC_MAX_DETECTABLE_AREA_PERCENTAGE
                              * 100.0
                              / 10.0))
                  * 10.0;

          latestMeasurements.add(maxFuelRealisticPercentage);
          while (latestMeasurements.size() > Constants.ObjectDetection.MAX_FUEL_GAUGE_MEASUREMENTS) {
            latestMeasurements.remove(0);
          }

          double avgRealisticPercentage = 0;

          if (!latestMeasurements.isEmpty()) {
            for (double i : latestMeasurements) {
              avgRealisticPercentage += i;
            }
            avgRealisticPercentage = avgRealisticPercentage / latestMeasurements.size();
          } else {
            avgRealisticPercentage = maxFuelRealisticPercentage;
          }

          DogLog.log("Vision/FuelGauge", maxFuelPercentage);
          DogLog.log("Vision/FuelGaugeRealistic", maxFuelRealisticPercentage);
          DogLog.log("Vision/AvgFuelGaugeRealistic", avgRealisticPercentage);

          logThresholdState(avgRealisticPercentage, maxFuelRealisticPercentage);
        },
        () -> DogLog.log("Vision/BlobPresent", false));
  }

  public void logThresholdState(double avgRealistic, double maxRealistic) {
    FuelGauge avgGauge, realisticGauge;

    if (avgRealistic < FuelGauge.EMPTY.getThreshold()) {
      avgGauge = FuelGauge.EMPTY;
    } else if (avgRealistic < FuelGauge.LOW.getThreshold()) {
      avgGauge = FuelGauge.LOW;
    } else if (avgRealistic < FuelGauge.MEDIUM.getThreshold()) {
      avgGauge = FuelGauge.MEDIUM;
    } else {
      avgGauge = FuelGauge.FULL;
    }

    if (maxRealistic < FuelGauge.EMPTY.getThreshold()) {
      realisticGauge = FuelGauge.EMPTY;
    } else if (maxRealistic < FuelGauge.LOW.getThreshold()) {
      realisticGauge = FuelGauge.LOW;
    } else if (maxRealistic < FuelGauge.MEDIUM.getThreshold()) {
      realisticGauge = FuelGauge.MEDIUM;
    } else {
      realisticGauge = FuelGauge.FULL;
    }

    DogLog.log("Vision/FuelGaugeLevel", avgGauge.toString());
    DogLog.log("Vision/FuelGaugeRealisticLevel", realisticGauge.toString());
  }

  public Optional<PhotonTrackedTarget> getLargestBlob() {
    if (latestVisionResult == null) return Optional.empty();
    List<PhotonTrackedTarget> targets = latestVisionResult.getTargets();
    if (targets.isEmpty()) return Optional.empty();

    return targets.stream().max((a, b) -> Double.compare(a.getArea(), b.getArea()));
  }

  public Optional<Double> getYawToBlob() {
    return getLargestBlob().map(PhotonTrackedTarget::getYaw);
  }

  public Optional<Double> getAreaOfBlob() {
    return getLargestBlob().map(PhotonTrackedTarget::getArea);
  }

  public Optional<Double> getPitchToBlob() {
    return getLargestBlob().map(PhotonTrackedTarget::getPitch);
  }

  public Optional<Double> getSkewToBlob() {
    return getLargestBlob().map(PhotonTrackedTarget::getSkew);
  }
}
