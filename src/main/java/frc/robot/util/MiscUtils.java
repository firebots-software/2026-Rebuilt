package frc.robot.util;

import dev.doglog.DogLog;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.io.File;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;

public class MiscUtils {
  private static int shiftIndicatorCounter = 0;

  public static Alliance getSecondAlliance() {
    Optional<String> allianceOpt = MatchState.getGameData();
    String allianceChar = allianceOpt.orElse("");
    if (allianceChar.isEmpty()) return null;
    return switch (allianceChar.charAt(0)) {
      case 'B' -> Alliance.BLUE;
      case 'R' -> Alliance.RED;
      default -> null;
    };
  }

  public static String activeFirst() {
    Optional<Alliance> alliance = MatchState.getAlliance();
    if (alliance.isEmpty() || MatchState.getMatchTime() < 105) return "";
    Alliance ourAlliance = alliance.get();
    Alliance secondAlliance = getSecondAlliance();
    if (secondAlliance == null) return "";
    return secondAlliance.equals(ourAlliance) ? "LATER" : "NOW";
  }
  public static boolean areWeActive(double currentMatchTime) {
    Optional<Alliance> alliance = MatchState.getAlliance();
    if (alliance.isEmpty()) return false;
    if (RobotState.isAutonomous()) return true;
    if (!RobotState.isTeleop()) return false;

    // transition + endgame
    if (currentMatchTime > 130.0 || currentMatchTime <= 30.0) return true;

    Alliance secondAlliance = getSecondAlliance();
    // falls back to red
    if (secondAlliance == null) secondAlliance = Alliance.RED;

    boolean weAreActiveFirst = (alliance.get() != secondAlliance);
    if (currentMatchTime > 105.0) {
      return weAreActiveFirst;
    } else if (currentMatchTime > 80.0) {
      return !weAreActiveFirst;
    } else if (currentMatchTime > 55.0) {
      return weAreActiveFirst;
    } else {
      return !weAreActiveFirst;
    }
  }

  public static boolean areWeActive() {
    return areWeActive(MatchState.getMatchTime());
  }

  public static double countdownTillNextShift(double currentMatchTime) {
    if (RobotState.isAutonomous()) {
      return Math.max(0.0, currentMatchTime);
    }
    if (currentMatchTime > 130.0) return currentMatchTime - 130.0;
    else if (currentMatchTime > 105.0) return currentMatchTime - 105.0;
    else if (currentMatchTime > 80.0) return currentMatchTime - 80.0;
    else if (currentMatchTime > 55.0) return currentMatchTime - 55.0;
    else if (currentMatchTime > 30.0) return currentMatchTime - 30.0;
    else return Math.max(0.0, currentMatchTime);
  }

  public static String currentShiftName(double currentMatchTime) {
    if (RobotState.isAutonomous()) return "Auto";
    if (currentMatchTime > 130.0) return "Transition";
    else if (currentMatchTime > 105.0) return "ALS 1";
    else if (currentMatchTime > 80.0) return "ALS 2";
    else if (currentMatchTime > 55.0) return "ALS 3";
    else if (currentMatchTime > 30.0) return "ALS 4";
    else return "Endgame";
  }

  public static void shiftSwitchIndicator(double currentMatchTime) {
    double timeUntilNextShift = countdownTillNextShift(currentMatchTime);
    String shiftName = currentShiftName(currentMatchTime);
    boolean isTransition = shiftName.equals("Transition");
    boolean isEndgame = shiftName.equals("Endgame");
    boolean isActive = areWeActive(currentMatchTime);

    if (isTransition || isEndgame) {
      shiftIndicatorCounter = 0;
      TelemetryUtils.elasticTelemetry.log("ShiftSwitchIndicator", "#00FF00");
      return;
    }

    shiftIndicatorCounter++;
    String color;

    if (isActive) {
      if (timeUntilNextShift >= 8.0) {
        color = "#00FF00";
      } else if (timeUntilNextShift < 2.0) {
        color = "#000000";
      } else if (timeUntilNextShift < 5.0) {
        color = ((shiftIndicatorCounter / 8) % 2 == 0) ? "#00FF00" : "#000000";
      } else {
        color = ((shiftIndicatorCounter / 20) % 2 == 0) ? "#00FF00" : "#000000";
      }
    } else {
      if (timeUntilNextShift >= 8.0) {
        color = "#000000";
      } else if (timeUntilNextShift < 2.0) {
        color = "#00FF00";
      } else if (timeUntilNextShift < 5.0) {
        color = ((shiftIndicatorCounter / 8) % 2 == 0) ? "#FFFF00" : "#000000";
      } else {
        color = ((shiftIndicatorCounter / 20) % 2 == 0) ? "#FFFF00" : "#000000";
      }
    }

    TelemetryUtils.elasticTelemetry.log("ShiftSwitchIndicator", color);
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
    // Check if /u/logs exists and is writable (indicates USB is actually mounted)
    File logsDir = new File("/u/logs");
    DogLog.log("Elastic/LogsDirExists", logsDir.exists());
    DogLog.log("Elastic/LogsDirIsDirectory", logsDir.isDirectory());
    DogLog.log("Elastic/LogsDirCanWrite", logsDir.canWrite());
    return logsDir.exists() && logsDir.isDirectory() && logsDir.canWrite();
  }

  public static File getFlashDriveDirectory() {
    File usbDrive = new File("/u");
    return usbDrive.exists() && usbDrive.isDirectory() ? usbDrive : null;
  }
}