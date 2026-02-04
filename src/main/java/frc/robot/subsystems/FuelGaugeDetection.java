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

  private static ArrayList<Double> latestRawMeasurements = new ArrayList<>();
  private static ArrayList<Double> latestMultipleMeasurements = new ArrayList<>();

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

          double rawArea = b.getArea();
          double smoothedRawArea = 0.0;
          double smoothedMultipleBalls = 0.0;

          updateLatestList(latestRawMeasurements, rawArea, smoothedRawArea);

          double avgMultipleBalls = getLargestBallsAvg(3);

          updateLatestList(latestMultipleMeasurements, avgMultipleBalls, smoothedMultipleBalls);

          DogLog.log("Subsystems/FuelGauge/RawArea", rawArea);
          DogLog.log("Subsystems/FuelGauge/SmoothedRawArea", smoothedRawArea);
          DogLog.log("Subsystems/FuelGauge/MultipleBallsArea", smoothedMultipleBalls);

          logThresholdState(smoothedRawArea, rawArea, smoothedMultipleBalls);
        },
        () -> DogLog.log("Subsystems/FuelGauge/BlobPresent", false));
  }

  private void updateLatestList(ArrayList<Double> list, double area, double smoothedArea) {
    list.add(area);
    while (list.size() > Constants.FuelGaugeDetection.MAX_FUEL_GAUGE_MEASUREMENTS) {
      list.remove(0);
    }

    if (!list.isEmpty()) {
      for (double i : list) {
        smoothedArea += i;
      }
      smoothedArea = smoothedArea / list.size();
    } else {
      smoothedArea = area;
    }
  }

  public void logThresholdState(double smoothedArea, double rawArea, double smoothedMultipleBalls) {
    FuelGauge smoothGauge, rawGauge, multipleBallsGauge;

    if (smoothedArea < FuelGauge.EMPTY.getThreshold()) {
      smoothGauge = FuelGauge.EMPTY;
    } else if (smoothedArea < FuelGauge.LOW.getThreshold()) {
      smoothGauge = FuelGauge.LOW;
    } else if (smoothedArea < FuelGauge.MEDIUM.getThreshold()) {
      smoothGauge = FuelGauge.MEDIUM;
    } else {
      smoothGauge = FuelGauge.FULL;
    }

    if (rawArea < FuelGauge.EMPTY.getThreshold()) {
      rawGauge = FuelGauge.EMPTY;
    } else if (rawArea < FuelGauge.LOW.getThreshold()) {
      rawGauge = FuelGauge.LOW;
    } else if (rawArea < FuelGauge.MEDIUM.getThreshold()) {
      rawGauge = FuelGauge.MEDIUM;
    } else {
      rawGauge = FuelGauge.FULL;
    }

    if (smoothedMultipleBalls < FuelGauge.EMPTY.getThreshold()) {
      multipleBallsGauge = FuelGauge.EMPTY;
    } else if (smoothedMultipleBalls < FuelGauge.LOW.getThreshold()) {
      multipleBallsGauge = FuelGauge.LOW;
    } else if (smoothedMultipleBalls < FuelGauge.MEDIUM.getThreshold()) {
      multipleBallsGauge = FuelGauge.MEDIUM;
    } else {
      multipleBallsGauge = FuelGauge.FULL;
    }

    DogLog.log("Subsystems/FuelGauge/SmoothedGaugeLevel", smoothGauge.toString());
    DogLog.log("Subsystems/FuelGauge/RawGaugeLevel", rawGauge.toString());
    DogLog.log("Subsystems/FuelGauge/MultipleBallsGaugeLevel", multipleBallsGauge.toString());
  }

  public Optional<PhotonTrackedTarget> getLargestBall() {
    if (latestVisionResult == null) return Optional.empty();
    List<PhotonTrackedTarget> targets = latestVisionResult.getTargets();
    if (targets.isEmpty()) return Optional.empty();

    return targets.stream().max((a, b) -> Double.compare(a.getArea(), b.getArea()));
  }

  public double getLargestBallsAvg(int numBalls) {
    double sum = 0.0;
    if (latestVisionResult == null) return 0.0;
    List<PhotonTrackedTarget> targets = latestVisionResult.getTargets();

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
