package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FuelGaugeDetection.FuelGauge;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class FuelGaugeDetection extends SubsystemBase {

  private static FuelGaugeDetection[] cameraList =
      new FuelGaugeDetection[Constants.Vision.Cameras.values().length];

  private static ArrayList<Double> latestMeasurements = new ArrayList<>();

  private final Constants.Vision.Cameras cameraID;
  private String cameraTitle;

  private final PhotonCamera photonCamera;
  private PhotonPipelineResult latestVisionResult;

  public FuelGaugeDetection(Constants.Vision.Cameras cameraID) {
    this.cameraID = cameraID;
    photonCamera = new PhotonCamera(cameraID.toString());

    cameraTitle = cameraID.getLoggingName();
  }

  public static FuelGaugeDetection getInstance(Constants.Vision.Cameras cameraID) {
    int index = cameraID.ordinal();
    if (cameraList[index] == null) cameraList[index] = new FuelGaugeDetection(cameraID);
    return cameraList[index];
  }

  public void periodic() {

    List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();
    for (var result : results) latestVisionResult = result;

    Optional<PhotonTrackedTarget> ball = getLargestBall();
    ball.ifPresentOrElse(
        b -> {
          DogLog.log("Subsystems/FuelGauge/BallPresent", true);
          DogLog.log("Subsystems/FuelGauge/BallYaw", b.getYaw());
          DogLog.log("Subsystems/FuelGauge/BallPitch", b.getPitch());
          DogLog.log("Subsystems/FuelGauge/BallSkew", b.getSkew());
          double maxFuelPercentage =
              ((double)
                      Math.round(
                          b.getArea()
                              / Constants.FuelGaugeDetection.MAX_DETECTABLE_FUEL_AREA_PERCENTAGE
                              * 100.0
                              / 10.0))
                  * 10.0;

          double maxFuelRealisticPercentage =
              ((double)
                      Math.round(
                          b.getArea()
                              / Constants.FuelGaugeDetection
                                  .REALISTIC_MAX_DETECTABLE_AREA_PERCENTAGE
                              * 100.0
                              / 10.0))
                  * 10.0;

          latestMeasurements.add(b.getArea());
          while (latestMeasurements.size()
              > Constants.FuelGaugeDetection.MAX_FUEL_GAUGE_MEASUREMENTS) {
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

          DogLog.log("Subsystems/FuelGauge/FuelGauge", maxFuelPercentage);
          DogLog.log("Subsystems/FuelGauge/FuelGaugeRealistic", maxFuelRealisticPercentage);
          DogLog.log("Subsystems/FuelGauge/AvgFuelGaugeRealistic", avgRealisticPercentage);

          logThresholdState(avgRealisticPercentage, b.getArea());
        },
        () -> DogLog.log("Subsystems/FuelGauge/BlobPresent", false));
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

    DogLog.log("Subsystems/FuelGauge/FuelGaugeLevel", avgGauge.toString());
    DogLog.log("Subsystems/FuelGauge/FuelGaugeRealisticLevel", realisticGauge.toString());
  }

  public Optional<PhotonTrackedTarget> getLargestBall() {
    if (latestVisionResult == null) return Optional.empty();
    List<PhotonTrackedTarget> targets = latestVisionResult.getTargets();
    if (targets.isEmpty()) return Optional.empty();

    return targets.stream().max((a, b) -> Double.compare(a.getArea(), b.getArea()));
  }

  public Optional<Double> getYawToBall() {
    return getLargestBall().map(PhotonTrackedTarget::getYaw);
  }

  public Optional<Double> getAreaOfBall() {
    return getLargestBall().map(PhotonTrackedTarget::getArea);
  }

  public Optional<Double> getPitchToBall() {
    return getLargestBall().map(PhotonTrackedTarget::getPitch);
  }

  public Optional<Double> getSkewToBall() {
    return getLargestBall().map(PhotonTrackedTarget::getSkew);
  }
}
