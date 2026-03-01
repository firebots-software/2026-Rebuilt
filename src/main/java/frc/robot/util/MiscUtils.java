package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.MathUtils.MiscMath;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public class MiscUtils {
  public static Pose2d plus(Pose2d a, Translation2d b) {
    return new Pose2d(
        a.getX() + b.getX(), a.getY() + b.getY(), new Rotation2d(a.getRotation().getRadians()));
  }

  public static Pose2d plusWithRotation(Pose2d a, Pose2d b) { // Transform2d used to be Pose2d
    return new Pose2d(
        a.getX() + b.getX(),
        a.getY() + b.getY(),
        new Rotation2d(a.getRotation().getRadians() + b.getRotation().getRadians()));
  }

  public static Alliance getSecondAlliance() {
    String allianceChar = DriverStation.getGameSpecificMessage();
    if (allianceChar.isEmpty()) return null;
    switch (allianceChar.charAt(0)) {
      case 'B':
        return Alliance.Blue;
      case 'R':
        return Alliance.Red;
      default:
        return null;
    }
  }

  public static boolean areWeActive(double currentTime) {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) return false;
    if (DriverStation.isAutonomousEnabled()) return true;
    if (!DriverStation.isTeleopEnabled()) return false;

    // teleop is enabled
    // double currentMatchTime = currentTime;
    double currentMatchTime = DriverStation.getMatchTime();
    String allianceChar = DriverStation.getGameSpecificMessage();

    if (allianceChar.isEmpty()) return true;
    boolean redInactiveFirst = getSecondAlliance() == Alliance.Red;

    boolean shift1Active =
        switch (alliance.get()) {
          case Red -> !redInactiveFirst;
          case Blue -> redInactiveFirst;
        };

    if (currentMatchTime > 130) return true;
    else if (currentMatchTime > 105) return shift1Active;
    else if (currentMatchTime > 80) return !shift1Active;
    else if (currentMatchTime > 55) return shift1Active;
    else if (currentMatchTime > 30) return !shift1Active;
    else return true;
  }

  public static double countdownTillNextShift(double currentTime) {
    // double currentMatchTime = currentTime;
    double currentMatchTime = DriverStation.getMatchTime();
    if (currentMatchTime > 140) return currentMatchTime - 140;
    else if (currentMatchTime > 130) return currentMatchTime - 130;
    else if (currentMatchTime > 105) return currentMatchTime - 105;
    else if (currentMatchTime > 80) return currentMatchTime - 80;
    else if (currentMatchTime > 55) return currentMatchTime - 55;
    else if (currentMatchTime > 30) return currentMatchTime - 30;
    else return 30 - currentMatchTime;
  }

  public static String currentShiftName(double currentTime) {
    double currentMatchTime = currentTime;
    //double currentMatchTime = DriverStation.getMatchTime();
    if (currentMatchTime > 140) return "Auto";
    else if (currentMatchTime > 130) return "Transition";
    else if (currentMatchTime > 105) return "ALS 1";
    else if (currentMatchTime > 80) return "ALS 2";
    else if (currentMatchTime > 55) return "ALS 3";
    else if (currentMatchTime > 30) return "ALS 4";
    else return "Endgame";
  }

  public static double getDistanceToHub(BooleanSupplier redSide, CommandSwerveDrivetrain swerve) {
    Pose2d robotPose = swerve.getCurrentState().Pose;

    Translation2d hubTranslation =
        redSide.getAsBoolean()
            ? new Translation2d(
                Constants.Landmarks.RED_HUB.getX(), Constants.Landmarks.RED_HUB.getY())
            : new Translation2d(
                Constants.Landmarks.BLUE_HUB.getX(), Constants.Landmarks.BLUE_HUB.getY());

    return robotPose.getTranslation().getDistance(hubTranslation);
  }

  public static double computeShootingSpeed(double distToHubCenter) {
    // Constants (meters)
    final double a = edu.wpi.first.math.util.Units.inchesToMeters(5.67405);
    final double b = edu.wpi.first.math.util.Units.inchesToMeters(36.60021);

    double y = (a * Math.sqrt(distToHubCenter)) + b;

    return MiscMath.clamp(y, 71.0, 107.0);
  }
}
