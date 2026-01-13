// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // TODO: Is our canbus still called this
  public static final String CANBUS_NAME = "Patrice the Pineapple";

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class Shooter {
    public static final MotorConstants motor1Constants = new MotorConstants(30);
    public static final MotorConstants motor2Constants = new MotorConstants(31);
    public static final MotorConstants preShooterConstants = new MotorConstants(32);

    public static final int ObjectDetectorPort = 1;

    public static Pose3d offset = new Pose3d(0,0,0, new Rotation3d()); //TODO: acquire vals

    public static float shootSpeedRps = 10f;
    public static float shooterWheelSize = 0f; //TODO: acquire vals
    public static float projectileInitSpeed = shootSpeedRps * shooterWheelSize * 2f * (float) Math.PI;
  }

  public static class MotorConstants {
    public int port;

    public MotorConstants(int port) {
      this.port = port;
    }
  }
}
