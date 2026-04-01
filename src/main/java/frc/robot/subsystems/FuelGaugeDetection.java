package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FuelGaugeDetection.FuelGauge;
import frc.robot.Constants.FuelGaugeDetection.GaugeCalculationType;
import frc.robot.util.VisionUtils;
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

  private double rawArea;
  private double smoothedArea;
  private double multipleBallsArea;
  private double smoothedMultipleBallsArea;

  private FuelGauge latestRawGauge;
  private FuelGauge latestSmoothedGauge;
  private FuelGauge latestMultipleBallsGauge;
  private FuelGauge latestSmoothedMultipleBallsGauge;

  public FuelGaugeDetection(Constants.FuelGaugeDetection.FuelGaugeCamera cameraID) {
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
    return results.isEmpty() ? false : results.get(results.size() - 1) == null;
  }

  private void updateVisionResult() {
    Optional<PhotonTrackedTarget> ball = getLargestBall();
    ball.ifPresentOrElse(
        b -> {
          DogLog.log("Subsystems/FuelGauge/BallPresent", true);
          DogLog.log("Subsystems/FuelGauge/BallYaw", b.getYaw());
          DogLog.log("Subsystems/FuelGauge/BallPitch", b.getPitch());
          DogLog.log("Subsystems/FuelGauge/BallSkew", b.getSkew());

          rawArea = b.getArea();
          smoothedArea = smoothFromList(latestRawMeasurements, rawArea);
          multipleBallsArea = getLargestBallsAvg(Constants.FuelGaugeDetection.BALLS_TO_AVG);
          smoothedMultipleBallsArea = smoothFromList(latestMultipleMeasurements, multipleBallsArea);

          DogLog.log("Subsystems/FuelGauge/Area/RawArea", rawArea);
          DogLog.log("Subsystems/FuelGauge/Area/SmoothedRawArea", smoothedArea);
          DogLog.log("Subsystems/FuelGauge/Area/MultipleBallsArea", multipleBallsArea);
          DogLog.log(
              "Subsystems/FuelGauge/Area/SmoothedMultipleBallsArea", smoothedMultipleBallsArea);

          calculateFuelGaugeState(
              rawArea, smoothedArea, multipleBallsArea, smoothedMultipleBallsArea);
        },
        () -> DogLog.log("Subsystems/FuelGauge/BallPresent", false));
  }

  private double smoothFromList(ArrayList<Double> list, double area) {
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

  private void calculateFuelGaugeState(
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

    SmartDashboard.putString(
        "Elastic/FuelGaugeLevel", VisionUtils.getColorOrDefault(latestSmoothedMultipleBallsGauge));
  }

  private FuelGauge setFuelGauge(double area) {
    if (area < FuelGauge.EMPTY.getThreshold()) return FuelGauge.EMPTY;
    if (area < FuelGauge.LOW.getThreshold()) return FuelGauge.LOW;
    if (area < FuelGauge.MEDIUM.getThreshold()) return FuelGauge.MEDIUM;
    return FuelGauge.FULL;
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
      default:
        return rawArea;
      case SMOOTHED:
        return smoothedArea;
      case MULTIPLE_BALLS:
        return multipleBallsArea;
      case SMOOTHED_MULTIPLE_BALLS:
        return smoothedMultipleBallsArea;
    }
  }

  public FuelGauge getGauge(GaugeCalculationType type) {
    switch (type) {
      case RAW:
      default:
        return latestRawGauge;
      case SMOOTHED:
        return latestSmoothedGauge;
      case MULTIPLE_BALLS:
        return latestMultipleBallsGauge;
      case SMOOTHED_MULTIPLE_BALLS:
        return latestSmoothedMultipleBallsGauge;
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

  public FuelGauge getCurrentFuelGaugeState() {
    double currentMeasurement = smoothedMultipleBallsArea; // or whichever measurement we use

    if (currentMeasurement <= FuelGauge.EMPTY.getThreshold()) return FuelGauge.EMPTY;
    if (currentMeasurement <= FuelGauge.LOW.getThreshold()) return FuelGauge.LOW;
    if (currentMeasurement <= FuelGauge.MEDIUM.getThreshold()) return FuelGauge.MEDIUM;
    return FuelGauge.FULL;
  }
}
