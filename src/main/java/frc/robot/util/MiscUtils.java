package frc.robot.util;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.io.File;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public class MiscUtils {

  public static int shiftIndicatorSum = 0;

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
    double currentMatchTime = currentTime;
    // double currentMatchTime = DriverStation.getMatchTime();
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
    // double currentMatchTime = DriverStation.getMatchTime();
    if (currentMatchTime > 140) return "Auto";
    else if (currentMatchTime > 130) return "Transition";
    else if (currentMatchTime > 105) return "ALS 1";
    else if (currentMatchTime > 80) return "ALS 2";
    else if (currentMatchTime > 55) return "ALS 3";
    else if (currentMatchTime > 30) return "ALS 4";
    else return "Endgame";
  }

  public static void shiftSwitchIndicator(double currentTime) {
    double currentTimes = currentTime;
    // double currentTimes = DriverStation.getMatchTime();
    double timeUntilNextShift = countdownTillNextShift(currentTimes);
    if (areWeActive(currentTimes) && !currentShiftName(currentTimes).equals("Endgame") && timeUntilNextShift > 8) {
      SmartDashboard.putString("Elastic/ShiftSwitchIndicator", "#00FF00");
    }
    else if (areWeActive(currentTimes) && !currentShiftName(currentTimes).equals("Endgame") && timeUntilNextShift < 8) {
      if ((shiftIndicatorSum / 20) % 2 == 1) {
        SmartDashboard.putString("Elastic/ShiftSwitchIndicator", "#000000");
      } else {
        SmartDashboard.putString("Elastic/ShiftSwitchIndicator", "#00FF00");
      }
      shiftIndicatorSum++;
    }
    else if ((timeUntilNextShift < 2 && !currentShiftName(currentTimes).equals("Endgame") && !areWeActive(currentTimes))) {
        SmartDashboard.putString("Elastic/ShiftSwitchIndicator", "#FFFFFF");
    }
    else if (timeUntilNextShift < 5 && !currentShiftName(currentTimes).equals("Endgame") && !areWeActive(currentTimes)) {
      if ((shiftIndicatorSum / 8) % 2 == 1) {
        SmartDashboard.putString("Elastic/ShiftSwitchIndicator", "#000000");
      } else {
        SmartDashboard.putString("Elastic/ShiftSwitchIndicator", "#FFFF00");
      }
      shiftIndicatorSum++;
    }
    else if (timeUntilNextShift < 8 && !currentShiftName(currentTimes).equals("Endgame") && !areWeActive(currentTimes)) {
      if ((shiftIndicatorSum / 20) % 2 == 1) {
        SmartDashboard.putString("Elastic/ShiftSwitchIndicator", "#000000");
      } else {
        SmartDashboard.putString("Elastic/ShiftSwitchIndicator", "#FFFFFF");
      }
      shiftIndicatorSum++;
    }
    else {
      shiftIndicatorSum = 0;
      SmartDashboard.putString("Elastic/ShiftSwitchIndicator", "#00FF00");
    }
  }

  public static double get3dDistance(Transform3d transform) {
    return Math.hypot(Math.hypot(transform.getX(), transform.getY()), transform.getZ());
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
    final double a = 20.41232;
    final double b = 57.26412;
    final double c = -0.182208;

    return (a * Math.sqrt(distToHubCenter - c)) + b;
  }

  public static boolean isFlashDriveConnected() {
    String[] possiblePaths = {"/u"};

    for (String path : possiblePaths) {
      // Check if /u/logs exists and is writable (indicates USB is actually mounted)
      File logsDir = new File(path + "/logs");
      DogLog.log("Elastic/logsDirExists", logsDir.exists());
      DogLog.log("Elastic/logsDirIsDirectory", logsDir.isDirectory());
      DogLog.log("Elastic/logsDirCanWrite", logsDir.canWrite());
      if (logsDir.exists() && logsDir.isDirectory() && logsDir.canWrite()) {
        return true;
      }
    }
    return false;
  }

  public static File getFlashDriveDirectory() {
    String[] possiblePaths = {"/u"};

    for (String path : possiblePaths) {
      File usbDrive = new File(path);
      if (usbDrive.exists() && usbDrive.isDirectory()) {
        return usbDrive;
      }
    }
    return null;
  }
}
