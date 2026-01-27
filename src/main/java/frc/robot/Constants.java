// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static final class Intake {
    public static final class Arm {
      public static final MotorConstants armMotor = new MotorConstants(34);

      public static final double armKV = 0.14;
      public static final double armKP = 0.1;
      public static final double armKI = 0;
      public static final double armKD = 0;

      public static final double armStatorCurrentLimit = 40.0;

      public static final int encoderPort = 0;

      public static final double armPosRetracted = 90.0;
      public static final double armPosExtended = 15.0;
      public static final double armPosIdle = 45.0; // subject to change

      public static final double MOTOR_ROTS_TO_ARM_ROTS = 1d / 77.8d; //MRD
      public static final double MOTOR_ROTS_TO_ARM_DEGREES = MOTOR_ROTS_TO_ARM_ROTS * 360d;
      public static final double ARM_DEGREES_TO_MOTOR_ROTS = 1d / MOTOR_ROTS_TO_ARM_DEGREES;
    }

    public static final MotorConstants intakeMotor = new MotorConstants(33);

    public static final double intakeKV = 0.14;
    public static final double intakeKP = 0.1;
    public static final double intakeKI = 0;
    public static final double intakeKD = 0;

    public static final double intakeSupplyCurrentLimit = 30.0;
    public static final double intakeStatorCurrentLimit = 50.0;
    public static final double intakeTargetSpeed = 40.0; // subject to change

    public static final double INTAKE_ROLLER_ROTS_TO_MOTOR_ROTS = 2.6666667d; //MRD
  }

  public static class MotorConstants {
    public int port;

    public MotorConstants(int port) {
      this.port = port;
    }
  }
}
