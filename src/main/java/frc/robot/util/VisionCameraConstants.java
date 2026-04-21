package frc.robot.util;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.math.util.Units;

public final class VisionCameraConstants {
  private static final class FrontRightVals {
    private static final double X = Units.inchesToMeters(-2.160666);
    private static final double Y = Units.inchesToMeters(-12.316702); // mrd negative
    private static final double Z = Units.inchesToMeters(26.967089);
    private static final double ROLL = Units.degreesToRadians(0.0);
    private static final double PITCH = Units.degreesToRadians(0.0);
    private static final double YAW = Units.degreesToRadians(-42.0); // mrd 310
    private static final double FX = 0.0; // cameramatrix
    private static final double CX = 0.0;
    private static final double FY = 0.0;
    private static final double CY = 0.0;
    private static final double K1 = 0.0; // distortioncoeffs
    private static final double K2 = 0.0;
    private static final double P1 = 0.0;
    private static final double P2 = 0.0;
    private static final double K3 = 0.0;
    private static final double K4 = 0.0;
    private static final double K5 = 0.0;
    private static final double K6 = 0.0;
  }

  public static final class FrontRight {
    public static final Transform3d transform =
        new Transform3d(
            new Translation3d(FrontRightVals.X, FrontRightVals.Y, FrontRightVals.Z),
            new Rotation3d(FrontRightVals.ROLL, FrontRightVals.PITCH, FrontRightVals.YAW));
    public static final Matrix<N3, N3> cameraMatrix =
        MatBuilder.fill(
            Nat.N3(),
            Nat.N3(),
            FrontRightVals.FX,
            0.0,
            FrontRightVals.CX,
            0.0,
            FrontRightVals.FY,
            FrontRightVals.CY,
            0.0,
            0.0,
            1.0);
    public static final Matrix<N8, N1> distCoeffs =
        MatBuilder.fill(
            Nat.N8(),
            Nat.N1(),
            FrontRightVals.K1,
            FrontRightVals.K2,
            FrontRightVals.P1,
            FrontRightVals.P2,
            FrontRightVals.K3,
            FrontRightVals.K4,
            FrontRightVals.K5,
            FrontRightVals.K6);
  }

  private static final class FrontLeftVals {
    private static final double X = Units.inchesToMeters(-2.160666);
    private static final double Y = Units.inchesToMeters(12.316702); // mrd positive
    private static final double Z = Units.inchesToMeters(26.967089);
    private static final double ROLL = Units.degreesToRadians(0.0);
    private static final double PITCH = Units.degreesToRadians(0.0);
    private static final double YAW = Units.degreesToRadians(42.0); // mrd 50
    private static final double FX = 0.0;
    private static final double CX = 0.0;
    private static final double FY = 0.0;
    private static final double CY = 0.0;
    private static final double K1 = 0.0;
    private static final double K2 = 0.0;
    private static final double P1 = 0.0;
    private static final double P2 = 0.0;
    private static final double K3 = 0.0;
    private static final double K4 = 0.0;
    private static final double K5 = 0.0;
    private static final double K6 = 0.0;
  }

  public static final class FrontLeft {
    public static final Transform3d transform =
        new Transform3d(
            new Translation3d(FrontLeftVals.X, FrontLeftVals.Y, FrontLeftVals.Z),
            new Rotation3d(FrontLeftVals.ROLL, FrontLeftVals.PITCH, FrontLeftVals.YAW));
    public static final Matrix<N3, N3> cameraMatrix =
        MatBuilder.fill(
            Nat.N3(),
            Nat.N3(),
            FrontLeftVals.FX,
            0.0,
            FrontLeftVals.CX,
            0.0,
            FrontLeftVals.FY,
            FrontLeftVals.CY,
            0.0,
            0.0,
            1.0);
    public static final Matrix<N8, N1> distCoeffs =
        MatBuilder.fill(
            Nat.N8(),
            Nat.N1(),
            FrontLeftVals.K1,
            FrontLeftVals.K2,
            FrontLeftVals.P1,
            FrontLeftVals.P2,
            FrontLeftVals.K3,
            FrontLeftVals.K4,
            FrontLeftVals.K5,
            FrontLeftVals.K6);
  }

  private static final class RearRightVals {
    private static final double X = Units.inchesToMeters(-14.106719);
    private static final double Y = Units.inchesToMeters(-11.537644); // mrd negative
    private static final double Z = Units.inchesToMeters(23.570693);
    private static final double ROLL = Units.degreesToRadians(0.0);
    private static final double PITCH = Units.degreesToRadians(330.321995);
    private static final double YAW = Units.degreesToRadians(148.35615); // mrd 328.35615
    private static final double FX = 0.0;
    private static final double CX = 0.0;
    private static final double FY = 0.0;
    private static final double CY = 0.0;
    private static final double K1 = 0.0;
    private static final double K2 = 0.0;
    private static final double P1 = 0.0;
    private static final double P2 = 0.0;
    private static final double K3 = 0.0;
    private static final double K4 = 0.0;
    private static final double K5 = 0.0;
    private static final double K6 = 0.0;
  }

  public static final class RearRight {
    public static final Transform3d transform =
        new Transform3d(
            new Translation3d(RearRightVals.X, RearRightVals.Y, RearRightVals.Z),
            new Rotation3d(RearRightVals.ROLL, RearRightVals.PITCH, RearRightVals.YAW));
    public static final Matrix<N3, N3> cameraMatrix =
        MatBuilder.fill(
            Nat.N3(),
            Nat.N3(),
            RearRightVals.FX,
            0.0,
            RearRightVals.CX,
            0.0,
            RearRightVals.FY,
            RearRightVals.CY,
            0.0,
            0.0,
            1.0);
    public static final Matrix<N8, N1> distCoeffs =
        MatBuilder.fill(
            Nat.N8(),
            Nat.N1(),
            RearRightVals.K1,
            RearRightVals.K2,
            RearRightVals.P1,
            RearRightVals.P2,
            RearRightVals.K3,
            RearRightVals.K4,
            RearRightVals.K5,
            RearRightVals.K6);
  }

  private static final class RearLeftVals {
    private static final double X = Units.inchesToMeters(-14.106719);
    private static final double Y = Units.inchesToMeters(11.537644); // mrd positive
    private static final double Z = Units.inchesToMeters(23.570693);
    private static final double ROLL = Units.degreesToRadians(0.0);
    private static final double PITCH = Units.degreesToRadians(330.321995);
    private static final double YAW = Units.degreesToRadians(211.64385); // mrd 31.643850
    private static final double FX = 0.0;
    private static final double CX = 0.0;
    private static final double FY = 0.0;
    private static final double CY = 0.0;
    private static final double K1 = 0.0;
    private static final double K2 = 0.0;
    private static final double P1 = 0.0;
    private static final double P2 = 0.0;
    private static final double K3 = 0.0;
    private static final double K4 = 0.0;
    private static final double K5 = 0.0;
    private static final double K6 = 0.0;
  }

  public static final class RearLeft {
    public static final Transform3d transform =
        new Transform3d(
            new Translation3d(RearLeftVals.X, RearLeftVals.Y, RearLeftVals.Z),
            new Rotation3d(RearLeftVals.ROLL, RearLeftVals.PITCH, RearLeftVals.YAW));
    public static final Matrix<N3, N3> cameraMatrix =
        MatBuilder.fill(
            Nat.N3(),
            Nat.N3(),
            RearLeftVals.FX,
            0.0,
            RearLeftVals.CX,
            0.0,
            RearLeftVals.FY,
            RearLeftVals.CY,
            0.0,
            0.0,
            1.0);
    public static final Matrix<N8, N1> distCoeffs =
        MatBuilder.fill(
            Nat.N8(),
            Nat.N1(),
            RearLeftVals.K1,
            RearLeftVals.K2,
            RearLeftVals.P1,
            RearLeftVals.P2,
            RearLeftVals.K3,
            RearLeftVals.K4,
            RearLeftVals.K5,
            RearLeftVals.K6);
  }
}
