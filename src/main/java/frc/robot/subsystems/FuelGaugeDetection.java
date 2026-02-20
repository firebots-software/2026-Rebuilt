package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FuelGaugeDetection.FuelGauge;
import frc.robot.Constants.FuelGaugeDetection.GaugeCalculationType;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class FuelGaugeDetection extends SubsystemBase {

  private static ArrayList<Double> latestRawMeasurements = new ArrayList<>();
  private static ArrayList<Double> latestMultipleMeasurements = new ArrayList<>();

  private final PhotonCamera photonCamera;
  private PhotonPipelineResult latestVisionResult;

  private double latestRawArea;
  private double latestSmoothedArea;
  private double latestMultipleBallsArea;
  private double latestSmoothedMultipleBallsArea;

  private FuelGauge latestRawGauge;
  private FuelGauge latestSmoothedGauge;
  private FuelGauge latestMultipleBallsGauge;
  private FuelGauge latestSmoothedMultipleBallsGauge;

  public FuelGaugeDetection(Constants.FuelGaugeDetection.FuelGaugeCamera cameraID) {
    photonCamera = new PhotonCamera(cameraID.toString());
  }

  public void periodic() {
    List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();
    for (PhotonPipelineResult result : results) latestVisionResult = result;

    if (latestVisionResult == null) {
      DogLog.log("Subsystems/FuelGauge/BallPresent", false);
      return;
    }

    Optional<PhotonTrackedTarget> ball = getLargestBall();
    ball.ifPresentOrElse(
        b -> {
          DogLog.log("Subsystems/FuelGauge/BallPresent", true);
          DogLog.log("Subsystems/FuelGauge/BallYaw", b.getYaw());
          DogLog.log("Subsystems/FuelGauge/BallPitch", b.getPitch());
          DogLog.log("Subsystems/FuelGauge/BallSkew", b.getSkew());

          double rawArea = b.getArea();
          double smoothedRawArea = updateLatestList(latestRawMeasurements, rawArea);
          double avgMultipleBalls = getLargestBallsAvg(Constants.FuelGaugeDetection.BALLS_TO_AVG);
          double smoothedMultipleBalls =
              updateLatestList(latestMultipleMeasurements, avgMultipleBalls);

          latestRawArea = rawArea;
          latestSmoothedArea = smoothedRawArea;
          latestMultipleBallsArea = avgMultipleBalls;
          latestSmoothedMultipleBallsArea = smoothedMultipleBalls;

          DogLog.log("Subsystems/FuelGauge/Area/RawArea", rawArea);
          DogLog.log("Subsystems/FuelGauge/Area/SmoothedRawArea", smoothedRawArea);
          DogLog.log("Subsystems/FuelGauge/Area/MultipleBallsArea", avgMultipleBalls);
          DogLog.log("Subsystems/FuelGauge/Area/SmoothedMultipleBallsArea", smoothedMultipleBalls);

          fuelGaugeStateSetAndLog(
              rawArea, smoothedRawArea, avgMultipleBalls, smoothedMultipleBalls);
        },
        () -> DogLog.log("Subsystems/FuelGauge/BallPresent", false));
  }

  private double updateLatestList(ArrayList<Double> list, double area) {
    double smoothedArea = 0.0;

    list.add(area);
    while (list.size() > Constants.FuelGaugeDetection.MAX_FUEL_GAUGE_MEASUREMENTS) {
      list.remove(0);
    }

    if (!list.isEmpty()) {
      for (double rawArea : list) smoothedArea += rawArea;
      smoothedArea /= list.size();
    } else smoothedArea = area;

    return smoothedArea;
  }

  private void fuelGaugeStateSetAndLog(
      double rawArea, double smoothedArea, double avgMultipleBalls, double smoothedMultipleBalls) {

    latestRawGauge = setFuelGauge(rawArea);
    latestSmoothedGauge = setFuelGauge(smoothedArea);
    latestMultipleBallsGauge = setFuelGauge(avgMultipleBalls);
    latestSmoothedMultipleBallsGauge = setFuelGauge(smoothedMultipleBalls);

    DogLog.log("Subsystems/FuelGauge/Gauge/RawGauge", latestRawGauge.toString());
    DogLog.log("Subsystems/FuelGauge/Gauge/SmoothedGauge", latestSmoothedGauge.toString());
    DogLog.log(
        "Subsystems/FuelGauge/Gauge/MultipleBallsGauge", latestMultipleBallsGauge.toString());
    DogLog.log(
        "Subsystems/FuelGauge/Gauge/SmoothedMultipleBallsGauge",
        latestSmoothedMultipleBallsGauge.toString());
  }

  private FuelGauge setFuelGauge(double area) {
    FuelGauge gauge;

    if (area < FuelGauge.EMPTY.getThreshold()) {
      gauge = FuelGauge.EMPTY;
    } else if (area < FuelGauge.LOW.getThreshold()) {
      gauge = FuelGauge.LOW;
    } else if (area < FuelGauge.MEDIUM.getThreshold()) {
      gauge = FuelGauge.MEDIUM;
    } else {
      gauge = FuelGauge.FULL;
    }

    return gauge;
  }

  private Optional<PhotonTrackedTarget> getLargestBall() {
    if (latestVisionResult == null) return Optional.empty();

    List<PhotonTrackedTarget> targets = latestVisionResult.getTargets();
    if (targets.isEmpty()) return Optional.empty();

    return targets.stream().max((a, b) -> Double.compare(a.getArea(), b.getArea()));
  }

  public double getArea(GaugeCalculationType type) {
    switch (type) {
      case RAW:
        return latestRawArea;
      case SMOOTHED:
        return latestSmoothedArea;
      case MULTIPLE_BALLS:
        return latestMultipleBallsArea;
      case SMOOTHED_MULTIPLE_BALLS:
        return latestSmoothedMultipleBallsArea;
      default:
        return latestRawArea;
    }
  }

  public FuelGauge getGauge(GaugeCalculationType type) {
    switch (type) {
      case RAW:
        return latestRawGauge;
      case SMOOTHED:
        return latestSmoothedGauge;
      case MULTIPLE_BALLS:
        return latestMultipleBallsGauge;
      case SMOOTHED_MULTIPLE_BALLS:
        return latestSmoothedMultipleBallsGauge;
      default:
        return latestRawGauge;
    }
  }

  public boolean GaugeLessThanEqualTo(GaugeCalculationType type, FuelGauge target) {
    return getGauge(type).getThreshold() <= target.getThreshold();
  }

  private double getLargestBallsAvg(int numBalls) {
    double sum = 0.0;
    if (latestVisionResult == null) return 0.0;
    List<PhotonTrackedTarget> targets = latestVisionResult.getTargets();
    if (targets.isEmpty()) return 0.0;

    numBalls = Math.min(numBalls, targets.size());

    for (int i = 0; i < numBalls; i++) {
      sum += targets.get(i).getArea();
    }

    return sum / numBalls;
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
