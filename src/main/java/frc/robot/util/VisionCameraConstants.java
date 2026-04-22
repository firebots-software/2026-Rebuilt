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
    private static final double FX = 913.9588182724549; // cameramatrix
    private static final double CX = 635.3113759543929;
    private static final double FY = 913.1110338032502;
    private static final double CY = 426.1145158880936;
    private static final double K1 = 0.053777333313074126; // distortioncoeffs
    private static final double K2 = -0.09163028264269202;
    private static final double P1 = 1.374201650684413E-4;
    private static final double P2 = 1.705990575776099E-4;
    private static final double K3 = 0.03043443776012655;
    private static final double K4 = -0.0014003230555085467;
    private static final double K5 = 0.003012682052878214;
    private static final double K6 = -0.0018355589720653578;
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
    private static final double FX = 912.5173606679848;
    private static final double CX = 641.6860955338284;
    private static final double FY = 911.7465912506361;
    private static final double CY = 352.76061130405736;
    private static final double K1 = 0.0510489487465398;
    private static final double K2 = -0.08377140729235197;
    private static final double P1 = 1.434092037672156E-4;
    private static final double P2 = -6.852290091675845E-4;
    private static final double K3 = 0.023945103433457356;
    private static final double K4 = -0.0027178370025379562;
    private static final double K5 = 0.005487814267694603;
    private static final double K6 = 7.918351577621345E-4;
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
    private static final double FX = 918.2654314246229;
    private static final double CX = 633.0888048530759;
    private static final double FY = 910.5580953980847;
    private static final double CY = 388.6942556055274;
    private static final double K1 = 0.04787004424953315;
    private static final double K2 = -0.06567381272383281;
    private static final double P1 = 3.7779485769898034E-4;
    private static final double P2 = -3.4797638844685785E-4;
    private static final double K3 = 0.009051433582570044;
    private static final double K4 = -0.002113451011579978;
    private static final double K5 = 0.005575990805806449;
    private static final double K6 = 0.001987724097504375;
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
    private static final double FX = 914.9774783104568;
    private static final double CX = 644.1129168082753;
    private static final double FY = 915.0443146858144;
    private static final double CY = 431.5500633760754;
    private static final double K1 = 0.04802570965310978;
    private static final double K2 = -0.07484429153153757;
    private static final double P1 = -6.419202733122086E-4;
    private static final double P2 = 3.908803817378362E-4;
    private static final double K3 = 0.01582645075158188;
    private static final double K4 = -0.0017780939236932693;
    private static final double K5 = 0.003169375124089103;
    private static final double K6 = -5.770679995204119E-4;
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

  private static final class FuelGaugeVals  {
    private static final double X = Units.inchesToMeters(-3.454827);
    private static final double Y = Units.inchesToMeters(-7.056897);
    private static final double Z = Units.inchesToMeters(25.105416);
    private static final double ROLL = Units.degreesToRadians(286.894287);
    private static final double PITCH = Units.degreesToRadians(55.646896);
    private static final double YAW = Units.degreesToRadians(23.957651);
  }

  public static final class FuelGauge {
    public static final Transform3d transform =new Transform3d(
              new Translation3d(FuelGaugeVals.X, FuelGaugeVals.Y, FuelGaugeVals.Z),
              new Rotation3d(FuelGaugeVals.ROLL, FuelGaugeVals.PITCH, FuelGaugeVals.YAW));
  }

  private static final class IntakeVisionVals {
    private static final double X = Units.inchesToMeters(0.0);
    private static final double Y = Units.inchesToMeters(22.342);
    private static final double Z = Units.inchesToMeters(18.9);
    private static final double ROLL = Units.degreesToRadians(0.0);
    private static final double PITCH = Units.degreesToRadians(9.789);
    private static final double YAW = Units.degreesToRadians(0.0);
  }

  public static final class IntakeVision {
    public static final Transform3d transform = new Transform3d(new Translation3d(IntakeVisionVals.X, IntakeVisionVals.Y, IntakeVisionVals.Z),
              new Rotation3d(IntakeVisionVals.ROLL, IntakeVisionVals.PITCH, IntakeVisionVals.YAW));
  }
}
