// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

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

  public static class Landmarks {

    public static final double MIDLINE_X = 0;
  }

  public static final class Shooter {
    public static final MotorConstants motor1Constants = new MotorConstants(35);
    public static final MotorConstants motor2Constants = new MotorConstants(34);
    public static final MotorConstants preShooterConstants = new MotorConstants(32);
  }

  public static final class Arm {
    public static final MotorConstants topLeftMotor = new MotorConstants(12);
    public static final MotorConstants topRightMotor = new MotorConstants(14);
    public static final MotorConstants bottomLeftMotor = new MotorConstants(11);
    public static final MotorConstants bottomRightMotor = new MotorConstants(13);

    public static final double conversionFactor = 130.63563333333335;
    public static final CANBus canbus = new CANBus("Patrice the Pineapple");

    public static double armKP = 1.0;
    public static double armKI = 0.0;
    public static double armKD = 0.0;
    public static final double armKS = 0.16969;
    public static final double armKG = 0.34;
    public static final double armKV = 2.49;

    public static final double absoluteEncoderHorizontal = 0.6400;
    public static final double absoluteHorizontalOffset = 0.05;
  }

  public static final class Intake {
    public static final int ObjectDetectorPort = 1;
    public static final MotorConstants intakeMotor = new MotorConstants(33);
    public static final double SUPPLY_CURRENT_LIMIT = 30.0;
    public static final double STATOR_CURRENT_LIMIT = 50.0;
  }

  public static class MotorConstants {
    public int port;

    public MotorConstants(int port) {
      this.port = port;
    }
  }
}
