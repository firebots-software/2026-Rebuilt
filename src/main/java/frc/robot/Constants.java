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

  public static enum SwerveDrivePIDValues {
    SERRANO(0.18014, 0d, 0d, -0.023265, 0.12681, 0.058864),
    PROTO(0.053218, 0d, 0d, 0.19977, 0.11198, 0.0048619),
    // JAMES_HARDEN(0.16901, 0d, 0d, 0.1593, 0.12143, 0.0091321); //0.041539 //0.12301
    JAMES_HARDEN(0.36, 0d, 0d, 0.2425, 0.11560693641, 0); // 0.041539 //0.12301
    public final double KP, KI, KD, KS, KV, KA;

    SwerveDrivePIDValues(double KP, double KI, double KD, double KS, double KV, double KA) {
      this.KP = KP;
      this.KI = KI;
      this.KD = KD;
      this.KS = KS;
      this.KV = KV;
      this.KA = KA;
    }
  }

  public static enum SwerveSteerPIDValues {
    SERRANO(50d, 0d, 0.2, 0d, 1.5, 0d),
    PROTO(20d, 0d, 0d, 0d, 0d, 0d),
    JAMES_HARDEN(38.982d, 2.4768d, 0d, 0.23791d, 0d, 0.1151d);
    public final double KP, KI, KD, KS, KV, KA;

    SwerveSteerPIDValues(double KP, double KI, double KD, double KS, double KV, double KA) {
      this.KP = KP;
      this.KI = KI;
      this.KD = KD;
      this.KS = KS;
      this.KV = KV;
      this.KA = KA;
    }
  }
}
