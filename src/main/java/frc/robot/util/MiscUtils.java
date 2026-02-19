package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

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

  public static boolean areWeActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) return false;
    if (DriverStation.isAutonomousEnabled()) return true;
    if (!DriverStation.isTeleopEnabled()) return false;

    // teleop is enabled
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
}
