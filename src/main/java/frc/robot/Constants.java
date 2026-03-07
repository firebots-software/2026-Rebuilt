package frc.robot;

import com.ctre.phoenix6.CANBus;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public final class Constants {
  public static final boolean hopperOnRobot = true;
  public static final boolean intakeOnRobot = true;
  public static final boolean visionOnRobot = true;
  public static final boolean fuelGaugeOnRobot = true;
  public static final boolean shooterOnRobot = true;
  public static final boolean climberOnRobot = true;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
