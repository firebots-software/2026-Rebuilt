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
  private class FrontRight {
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
    private static final double K1 = 0.0; // distancecoeffs
    private static final double K2 = 0.0;
    private static final double P1 = 0.0;
    private static final double P2 = 0.0;
    private static final double K3 = 0.0;
    private static final double K4 = 0.0;
    private static final double K5 = 0.0;
    private static final double K6 = 0.0;
  }

  public Transform3d frontRightTransform =
      new Transform3d(
          new Translation3d(FrontRight.X, FrontRight.Y, FrontRight.Z),
          new Rotation3d(FrontRight.ROLL, FrontRight.PITCH, FrontRight.YAW));
  public Matrix<N3, N3> frontRightCameraMatrix =
      MatBuilder.fill(
          Nat.N3(),
          Nat.N3(),
          FrontRight.FX,
          0.0,
          FrontRight.CX,
          0.0,
          FrontRight.FY,
          FrontRight.CY,
          0.0,
          0.0,
          1.0);
  public Matrix<N8, N1> frontRightDistCoeffs =
      MatBuilder.fill(
          Nat.N8(),
          Nat.N1(),
          FrontRight.K1,
          FrontRight.K2,
          FrontRight.P1,
          FrontRight.P2,
          FrontRight.K3,
          FrontRight.K4,
          FrontRight.K5,
          FrontRight.K6);

  private class FrontLeft {
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

  public Transform3d frontLeftTransform =
      new Transform3d(
          new Translation3d(FrontLeft.X, FrontLeft.Y, FrontLeft.Z),
          new Rotation3d(FrontLeft.ROLL, FrontLeft.PITCH, FrontLeft.YAW));
  public Matrix<N3, N3> frontLeftCameraMatrix =
      MatBuilder.fill(
          Nat.N3(),
          Nat.N3(),
          FrontLeft.FX,
          0.0,
          FrontLeft.CX,
          0.0,
          FrontLeft.FY,
          FrontLeft.CY,
          0.0,
          0.0,
          1.0);
  public Matrix<N8, N1> frontLeftDistCoeffs =
      MatBuilder.fill(
          Nat.N8(),
          Nat.N1(),
          FrontLeft.K1,
          FrontLeft.K2,
          FrontLeft.P1,
          FrontLeft.P2,
          FrontLeft.K3,
          FrontLeft.K4,
          FrontLeft.K5,
          FrontLeft.K6);

  private class RearRight {
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

  public Transform3d rearRightTransform =
      new Transform3d(
          new Translation3d(RearRight.X, RearRight.Y, RearRight.Z),
          new Rotation3d(RearRight.ROLL, RearRight.PITCH, RearRight.YAW));
  public Matrix<N3, N3> rearRightCameraMatrix =
      MatBuilder.fill(
          Nat.N3(),
          Nat.N3(),
          RearRight.FX,
          0.0,
          RearRight.CX,
          0.0,
          RearRight.FY,
          RearRight.CY,
          0.0,
          0.0,
          1.0);
  public Matrix<N8, N1> rearRightDistCoeffs =
      MatBuilder.fill(
          Nat.N8(),
          Nat.N1(),
          RearRight.K1,
          RearRight.K2,
          RearRight.P1,
          RearRight.P2,
          RearRight.K3,
          RearRight.K4,
          RearRight.K5,
          RearRight.K6);

  private class RearLeft {
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

  public Transform3d rearLeftTransform =
      new Transform3d(
          new Translation3d(RearLeft.X, RearLeft.Y, RearLeft.Z),
          new Rotation3d(RearLeft.ROLL, RearLeft.PITCH, RearLeft.YAW));
  public Matrix<N3, N3> rearLeftCameraMatrix =
      MatBuilder.fill(
          Nat.N3(),
          Nat.N3(),
          RearLeft.FX,
          0.0,
          RearLeft.CX,
          0.0,
          RearLeft.FY,
          RearLeft.CY,
          0.0,
          0.0,
          1.0);
  public Matrix<N8, N1> rearLeftDistCoeffs =
      MatBuilder.fill(
          Nat.N8(),
          Nat.N1(),
          RearLeft.K1,
          RearLeft.K2,
          RearLeft.P1,
          RearLeft.P2,
          RearLeft.K3,
          RearLeft.K4,
          RearLeft.K5,
          RearLeft.K6);
}
