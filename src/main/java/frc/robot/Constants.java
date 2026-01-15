// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Vision {

    // initializes cameras for use in VisionSubsystem
    public static enum Cameras {
      RIGHT_CAM("rightCam"),
      LEFT_CAM("leftCam");

      private String loggingName;

      Cameras(String name) {
        loggingName = name;
      }

      public String getLoggingName() {
        return loggingName;
      }
    }

    // Constants for noise calculation
    public static final double DISTANCE_EXPONENTIAL_COEFFICIENT_X = 0.00046074;
    public static final double DISTANCE_EXPONENTIAL_BASE_X = 2.97294;
    public static final double DISTANCE_EXPONENTIAL_COEFFICIENT_Y = 0.0046074;
    public static final double DISTANCE_EXPONENTIAL_BASE_Y = 2.97294;

    public static final double DISTANCE_COEFFICIENT_THETA = 0.9;

    public static final double ANGLE_COEFFICIENT_X =
        0.5; // noise growth per radian of viewing angle
    public static final double ANGLE_COEFFICIENT_Y = 0.5;
    public static final double ANGLE_COEFFICIENT_THETA = 0.5;

    public static final double SPEED_COEFFICIENT_X = 0.5; // noise growth per fraction of max speed
    public static final double SPEED_COEFFICIENT_Y = 0.5;
    public static final double SPEED_COEFFICIENT_THETA = 0.5;

    // placeholder constants for now; will be updated once robot is delivered
    public static final double RIGHT_X = Units.inchesToMeters(8.867);
    public static final double RIGHT_Y = Units.inchesToMeters(-12.4787);
    public static final double RIGHT_Z = Units.inchesToMeters(6.158);
    public static final double RIGHT_ROLL = Units.degreesToRadians(0.0);
    public static final double RIGHT_PITCH = Units.degreesToRadians(-12.5);
    public static final double RIGHT_YAW = Units.degreesToRadians(40);

    public static final double LEFT_X = Units.inchesToMeters(8.867);
    public static final double LEFT_Y = Units.inchesToMeters(12.478);
    public static final double LEFT_Z = Units.inchesToMeters(6.158);
    public static final double LEFT_ROLL = Units.degreesToRadians(0.0);
    public static final double LEFT_PITCH = Units.degreesToRadians(-12.5);
    public static final double LEFT_YAW = Units.degreesToRadians(-40);

    // initializing Transform3d for use in future field visualization
    public static Transform3d getCameraTransform(Cameras camera) {
      switch (camera) {
        case RIGHT_CAM:
          return new Transform3d(
              new Translation3d(RIGHT_X, RIGHT_Y, RIGHT_Z),
              new Rotation3d(RIGHT_ROLL, RIGHT_PITCH, RIGHT_YAW));
        case LEFT_CAM:
          return new Transform3d(
              new Translation3d(LEFT_X, LEFT_Y, LEFT_Z),
              new Rotation3d(LEFT_ROLL, LEFT_PITCH, LEFT_YAW));
        default:
          throw new IllegalArgumentException("Unknown camera ID: " + camera);
      }
    }
  }
}
