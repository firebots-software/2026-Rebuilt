package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.Pooer.ShooterType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Kalman {
    public static final Matrix<N3, N1> visionMatrix = VecBuilder.fill(0.01, 0.03d, 100d);
    public static final Matrix<N3, N1> odometryMatrix = VecBuilder.fill(0.1, 0.1, 0.1);
  }

  public static class HardenConstants {
    public static class EndWhenCloseEnough {
      public static final double translationalToleranceTeleop = 0.8d; // 0.43105229381 worked before
      public static final double translationalToleranceAuto = 1d;
      // public static final double translationalTolerance = 0.6;
      public static final double headingTolerance = 0.7853975; // Math.PI/4
    }

    public static final double ffMinRadius = 0.4; // 0.2 worked good
    public static final double ffMaxRadius = 1.4; // 0.8 worked good

    public static class RegularCommand {
      public static final double xyIndividualTolerance = 0.02;
      public static final double headingTolerance = 0.0075;
    }
  }

  public static class LED {
    public static final int LED_STRIP_LENGTH = 34;
    public static final int LED_STRIP_PORT = 7;
    public static final int[] PURE_RED = {0, 100, 100};
    public static final int[] PURE_BLUE = {201, 100, 100};
    public static final int[] PURE_YELLOW = {61, 100, 100};
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int PS4_CONTROLLER_PORT_1 = 3;
    public static final int SQUARE_BUTTON_PORT = 1;
    public static final int X_BUTTON_PORT = 2;
    public static final int CIRCLE_BUTTON_PORT = 3;
    public static final int TRIANGLE_BUTTON_PORT = 4;
    public static final int L1_BUTTON_PORT = 5;
    public static final int R1_BUTTON_PORT = 6;
    public static final int L2_BUTTON_PORT = 7;
    public static final int R2_BUTTON_PORT = 8;
    public static final int PS_SHARE_BUTTON_PORT = 9;
    public static final int OPTIONS_BUTTON_PORT = 10;
    public static final int L3_BUTTON_PORT = 11;
    public static final int R3_BUTTON_PORT = 12;
    public static final int PS_BUTTON_PORT = 13;
    public static final int BIG_BUTTON_PORT = 14;
  }

  public static final class MotorConstants {
    public final int PORT;
    public final boolean REVERSED;
    public final double GEAR_RATIO;
    public final double STATOR_CURRENT_LIMIT_AMPS;
    public final double SPEED_RPS;
    public final double AMP_SPEED_RPS;
    public final double SPEED_VOLTAGE;

    private MotorConstants(
        int port,
        boolean reversed,
        double gearRatio,
        double statorCurrent,
        double speed,
        double ampSpeed,
        double voltage) {
      PORT = port;
      REVERSED = reversed;
      GEAR_RATIO = gearRatio;
      STATOR_CURRENT_LIMIT_AMPS = statorCurrent;
      SPEED_RPS = speed;
      AMP_SPEED_RPS = ampSpeed;
      SPEED_VOLTAGE = voltage;
    }

    public static MotorConstants speedControled(
        int port,
        boolean reversed,
        double gearRatio,
        double statorCurrent,
        double speed,
        double ampSpeed) {
      return new MotorConstants(port, reversed, gearRatio, statorCurrent, speed, ampSpeed, 0);
    }

    public static MotorConstants voltageControlled(
        int port,
        boolean reversed,
        double gearRatio,
        double statorCurrent,
        double ampSpeed,
        double voltage) {
      return new MotorConstants(port, reversed, gearRatio, statorCurrent, 0, ampSpeed, voltage);
    }

    public static MotorConstants dualControlled(
        int port,
        boolean reversed,
        double gearRatio,
        double statorCurrent,
        double ampSpeed,
        double speed,
        double voltage) {
      return new MotorConstants(port, reversed, gearRatio, statorCurrent, speed, ampSpeed, voltage);
    }
  }

  public static final class Pooer {
    public static final int NOTE_DETECTOR_PORT = 1;
    public static final String CANBUS_NAME = "rio";

    public static final ShooterType SHOOTER = ShooterType.PIPER;

    public static enum ShooterType {
      PETER(
          MotorConstants.speedControled(30, true, 12d / 15d, 40.0, 4500d / 60d, 4500d / 60d),
          MotorConstants.speedControled(31, false, 12d / 15d, 40.0, 4500d / 60d, 4500d / 60d),
          MotorConstants.dualControlled(32, false, 4d / 1d, 45.0, 0, 300 / 60, 9d),
          MotorConstants.speedControled(33, true, 2d / 1d, 50.0, 200d, 200d)),
      PIPER(
          MotorConstants.speedControled(35, false, 24d / 18d, 40.0, 2000d / 60d, 2250d / 60d), //3500 / 60 was prev val
          MotorConstants.speedControled(34, false, 24d / 18d, 40.0, 2000d / 60d, 2250d / 60d), //3500 / 60 was prev val
          MotorConstants.dualControlled(32, true, 4d / 1d, 45.0, 0, 300 / 60, 9d),
          MotorConstants.speedControled(33, true, 2d / 1d, 50.0, 200d, 200d));
      public final MotorConstants SHOOTER_1, SHOOTER_2, PRESHOOTER, INTAKE;

      ShooterType(
          MotorConstants shooter1,
          MotorConstants shooter2,
          MotorConstants preshooter,
          MotorConstants intake) {
        SHOOTER_1 = shooter1;
        SHOOTER_2 = shooter2;
        PRESHOOTER = preshooter;
        INTAKE = intake;
      }
    }
  }

  public static final class Arm {
    public static final double BUNDT_ANGLE = 12d;
    public static final double ARM_STATOR_CURRENT_LIMIT_AMPS = 40.0;
    public static final double DEFAULT_ARM_ANGLE = 56.12;
    public static final double INTAKE_ANGLE = 3; // subject to change
    public static final double AMP_ANGLE = 95; // subject to change
    // public static final double ARM_ENCODER_OFFSET = 0; // TODO: Change the offset so that the 0
    // position is when the arm is at its resting
    // position.

    // TODO: Is our canbus still called this?
    public static final String CANBUS_NAME = "Patrice the Pineapple";

    public static final int RT_PORT = 14; // Right Top motor
    public static final int RB_PORT = 13; // Right Bottom motor
    public static final int LT_PORT = 12; // Left Top motor
    public static final int LB_PORT = 11; // Left Bottom motor
    public static final int ENCODER_PORT = 0; // subject to change

    public static final double CURRENT_LIMIT = 8.0;
    public static final double S0C_KP = 1.0;
    public static final double ARMFF_KS = 0.16969;
    public static final double ARMFF_KG = 0.34;
    public static final double ARMFF_KV = 2.49;
    public static final double MOTIONMAGIC_KV = 1; // MotionMagic Cruise Velocity in RPS of the arm
    public static final double MOTIONMAGIC_KA = 2.2; // MotionMagic Acceleration in RPS^2 of the arm

    public static final double FEET_TO_METERS_CONVERSION_FACTOR = 0.3048;
    public static final double ABSOLUTE_ARM_CONVERSION_FACTOR = 42d / 18d;
    public static final double INTEGRATED_ABSOLUTE_CONVERSION_FACTOR = 55.9867;
    public static final double INTEGRATED_ARM_CONVERSION_FACTOR =
        ABSOLUTE_ARM_CONVERSION_FACTOR
            * INTEGRATED_ABSOLUTE_CONVERSION_FACTOR; // 130.63563333333335;
    public static final double ABSOLUTE_ENCODER_HORIZONTAL = 0.6655; // 0.6547
    public static final double ABSOLUTE_HORIZONTAL_OFFSET = 0.05; // 0.05
    public static double ARM_INTERMAP_OFFSET = 0;
    // public static double ZERO_SPEAKER_OFFSET_METERS = 0.6;
    public static final InterpolatingDoubleTreeMap INTERMAP = new InterpolatingDoubleTreeMap();

    static {
      UPDATE_INTERMAP();
    }

    public static void UPDATE_INTERMAP() {
      if (Constants.Pooer.SHOOTER == ShooterType.PETER) {
        UPDATE_INTERMAP_PETER();
      } else {
        UPDATE_INTERMAP_PIPER();
      }
    }

    public static void UPDATE_INTERMAP_PETER() {
      INTERMAP.clear();
      INTERMAP.put(
          1.34,
          6.5 + ARM_INTERMAP_OFFSET); // measurements of distance are from front of robot bumper to
      // wall
      INTERMAP.put(2.1, 17d + ARM_INTERMAP_OFFSET);
      INTERMAP.put(Units.feetToMeters(9) + Units.inchesToMeters(17), 23.5d + ARM_INTERMAP_OFFSET);
    }

    public static void UPDATE_INTERMAP_PIPER() {
      INTERMAP.clear();
      INTERMAP.put(1.34, 6.46 + ARM_INTERMAP_OFFSET);
      INTERMAP.put(1.34 + Units.inchesToMeters(30), 20.6 + ARM_INTERMAP_OFFSET);
      INTERMAP.put(1.34 + Units.inchesToMeters(60), 27.8 + ARM_INTERMAP_OFFSET);
      INTERMAP.put(1.34 + Units.inchesToMeters(90), 31.339 + ARM_INTERMAP_OFFSET);
      INTERMAP.put(1.34 + Units.inchesToMeters(120), 32.67 + ARM_INTERMAP_OFFSET);
    }
  }

  public static class OI {
    public static final double LEFT_JOYSTICK_DEADBAND = 0.07;
    public static final double RIGHT_JOYSTICK_DEADBAND = 0.07;
    public static final int JOYSTICK_A_PORT = 0;

    public enum XBoxButtonID {
      /** A. */
      A(1),
      /** B. */
      B(2),
      /** X. */
      X(3),
      /** Y. */
      Y(4),
      /** Left bumper. */
      LeftBumper(5),
      /** Right bumper. */
      RightBumper(6),
      /** Left stick. */
      LeftStick(9),
      /** Right stick. */
      RightStick(10),
      /** Back. */
      Back(7),
      /** Start. */
      Start(8);
      public final int value;

      XBoxButtonID(int value) {
        this.value = value;
      }
    }

    public enum AxisID {
      /** Left X. */
      LeftX(0),
      /** Right X. */
      RightX(4),
      /** Left Y. */
      LeftY(1),
      /** Right Y. */
      RightY(5),
      /** Left trigger. */
      LeftTrigger(2),
      /** Right trigger. */
      RightTrigger(3);

      /** Axis value. */
      public final int value;

      AxisID(int value) {
        this.value = value;
      }
    }
  }

  public static class Swerve {
    public static final SwerveType WHICH_SWERVE_ROBOT = SwerveType.SERRANO;

    public static enum SwerveLevel {
      L2(6.75, 21.428571428571427),
      L3(6.12, 21.428571428571427);
      public final double DRIVE_GEAR_RATIO, STEER_GEAR_RATIO;

      SwerveLevel(double drive, double steer) {
        DRIVE_GEAR_RATIO = drive;
        STEER_GEAR_RATIO = steer;
      }
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

    public static enum RobotDimensions {
      SERRANO(Inches.of(22.52), Inches.of(22.834)), // length, width
      PROTO(Inches.of(22.52), Inches.of(22.834)), // length, width
      JAMES_HARDEN(Inches.of(26.75), Inches.of(22.75)); // length, width
      public final Distance length, width;

      RobotDimensions(Distance length, Distance width) {
        this.length = length;
        this.width = width;
      }
    }

    public static enum BumperThickness {
      SERRANO(Inches.of(2.625)), // thickness
      PROTO(Inches.of(2.625)), // thickness
      JAMES_HARDEN(Inches.of(3.313)); // thickness
      public final Distance thickness;

      BumperThickness(Distance thickness) {
        this.thickness = thickness;
      }
    }

    public static enum SwerveType {
      SERRANO(
          Rotations.of(-0.466552734375), // front left
          Rotations.of(-0.436767578125), // front right
          Rotations.of(-0.165283203125), // back left
          Rotations.of(-0.336181640625), // back right
          SwerveLevel.L3, // what level the swerve drive is
          SwerveDrivePIDValues.SERRANO,
          SwerveSteerPIDValues.SERRANO,
          RobotDimensions.SERRANO,
          "Patrice the Pineapple",
          BumperThickness.SERRANO),
      PROTO(
          Rotations.of(0.3876953125), // front left
          Rotations.of(0.159912109375), // front right
          Rotations.of(0.213134765625), // back left
          Rotations.of(-0.3818359375), // back right
          SwerveLevel.L2, // what level the swerve drive is
          SwerveDrivePIDValues.PROTO,
          SwerveSteerPIDValues.PROTO,
          RobotDimensions.PROTO,
          "rio",
          BumperThickness.PROTO),
      JAMES_HARDEN(
          Rotations.of(-0.0834960938), // front left
          Rotations.of(-0.4912109375), // front right
          Rotations.of(0.1931152344), // back left
          Rotations.of(-0.15576171875), // back right
          SwerveLevel.L3,
          SwerveDrivePIDValues.JAMES_HARDEN,
          SwerveSteerPIDValues.JAMES_HARDEN,
          RobotDimensions.JAMES_HARDEN,
          "JamesHarden",
          BumperThickness.JAMES_HARDEN);
      public final Angle FRONT_LEFT_ENCODER_OFFSET,
          FRONT_RIGHT_ENCODER_OFFSET,
          BACK_LEFT_ENCODER_OFFSET,
          BACK_RIGHT_ENCODER_OFFSET;
      public final SwerveLevel SWERVE_LEVEL;
      public final SwerveDrivePIDValues SWERVE_DRIVE_PID_VALUES;
      public final SwerveSteerPIDValues SWERVE_STEER_PID_VALUES;
      public final RobotDimensions ROBOT_DIMENSIONS;
      public final String CANBUS_NAME;
      public final BumperThickness BUMPER_THICKNESS;

      SwerveType(
          Angle fl,
          Angle fr,
          Angle bl,
          Angle br,
          SwerveLevel swerveLevel,
          SwerveDrivePIDValues swerveDrivePIDValues,
          SwerveSteerPIDValues swerveSteerPIDValues,
          RobotDimensions robotDimensions,
          String canbus_name,
          BumperThickness thickness) {
        FRONT_LEFT_ENCODER_OFFSET = fl;
        FRONT_RIGHT_ENCODER_OFFSET = fr;
        BACK_LEFT_ENCODER_OFFSET = bl;
        BACK_RIGHT_ENCODER_OFFSET = br;
        SWERVE_LEVEL = swerveLevel;
        SWERVE_DRIVE_PID_VALUES = swerveDrivePIDValues;
        SWERVE_STEER_PID_VALUES = swerveSteerPIDValues;
        ROBOT_DIMENSIONS = robotDimensions;
        CANBUS_NAME = canbus_name;
        BUMPER_THICKNESS = thickness;
      }
    }

    public static class Simulation {
      // These are only used for simulation
      private static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.01);
      private static final MomentOfInertia DRIVE_INERTIA = KilogramSquareMeters.of(0.01);
      // Simulated voltage necessary to overcome friction
      private static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.2);
      private static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.2);
    }

    // TODO: Tune the Steer and Drive gains using SysID
    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs STEER_GAINS =
        new Slot0Configs()
            .withKP(WHICH_SWERVE_ROBOT.SWERVE_STEER_PID_VALUES.KP)
            .withKI(WHICH_SWERVE_ROBOT.SWERVE_STEER_PID_VALUES.KI)
            .withKD(WHICH_SWERVE_ROBOT.SWERVE_STEER_PID_VALUES.KD)
            .withKS(WHICH_SWERVE_ROBOT.SWERVE_STEER_PID_VALUES.KS)
            .withKV(WHICH_SWERVE_ROBOT.SWERVE_STEER_PID_VALUES.KV)
            .withKA(WHICH_SWERVE_ROBOT.SWERVE_STEER_PID_VALUES.KA);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs DRIVE_GAINS =
        new Slot0Configs()
            .withKP(WHICH_SWERVE_ROBOT.SWERVE_DRIVE_PID_VALUES.KP)
            .withKI(WHICH_SWERVE_ROBOT.SWERVE_DRIVE_PID_VALUES.KI)
            .withKD(WHICH_SWERVE_ROBOT.SWERVE_DRIVE_PID_VALUES.KD)
            .withKS(WHICH_SWERVE_ROBOT.SWERVE_DRIVE_PID_VALUES.KS)
            .withKV(WHICH_SWERVE_ROBOT.SWERVE_DRIVE_PID_VALUES.KV)
            .withKA(WHICH_SWERVE_ROBOT.SWERVE_DRIVE_PID_VALUES.KA);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT =
        ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT =
        ClosedLoopOutputType.Voltage;

    // The type of motor used for the drive motor
    private static final DriveMotorArrangement DRIVE_MOTOR_TYPE =
        DriveMotorArrangement.TalonFX_Integrated;
    // The type of motor used for the drive motor
    private static final SteerMotorArrangement STEER_MOTOR_TYPE =
        SteerMotorArrangement.TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to
    //  RemoteCANcoder
    private static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    // This is where we apply Current Limits for swerve
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private static final TalonFXConfiguration DRIVE_INITIAL_CONFIGS =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(90.0))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(50.0)) // 40.0
                    .withSupplyCurrentLimitEnable(true));
    private static final TalonFXConfiguration STEER_INITIAL_CONFIGS =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(40)
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(30)));

    private static final CANcoderConfiguration ENCODER_INITIAL_CONFIGS =
        new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    // TODO: investigate Pigeon2Configuration and how it's relevant
    private static final Pigeon2Configuration PIGEON2_CONFIGS = null;

    // TODO: CHANGE FOR NEW ROBOT
    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    public static final CANBus CANBUS_NAME = new CANBus(WHICH_SWERVE_ROBOT.CANBUS_NAME);

    // TODO: VERIFY FOR NEW ROBOT
    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final Current SLIP_CURRENT_AMPS = Amps.of(100.0);

    public static final Current DRIVE_STATOR_CURRENT_LIMIT_AMPS = Amps.of(90.0);
    public static final Current STEER_STATOR_CURRENT_LIMIT_AMPS = Amps.of(40.0);

    public static final Current DRIVE_SUPPLY_CURRENT_LIMIT_AMPS = Amps.of(40.0);
    public static final Current TURNING_SUPPLY_CURRENT_LIMIT_AMPS = Amps.of(30.0);

    public static final Current DUTY_CYCLE_VELOCITY = Current.ofBaseUnits(30.0, Amp);
    public static final Current ACCELERATION = Current.ofBaseUnits(50.0, Amp);

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final LinearVelocity SPEED_AT_12V_METERS_PER_SECOND =
        MetersPerSecond.of(4.73); // TODO: VERIFY FOR NEW ROBOT

    private static final double COUPLE_RATIO = 3.5714285714285716;

    private static final double DRIVE_GEAR_RATIO =
        WHICH_SWERVE_ROBOT.SWERVE_LEVEL.DRIVE_GEAR_RATIO; // TODO: VERIFY FOR NEW ROBOT
    private static final double STEER_GEAR_RATIO =
        WHICH_SWERVE_ROBOT.SWERVE_LEVEL.STEER_GEAR_RATIO; // TODO: VERIFY FOR NEW ROBOT
    private static final Distance WHEEL_RADIUS_INCHES = Inches.of(2); // TODO: VERIFY FOR NEW ROBOT

    private static final boolean STEER_MOTOR_REVERSED = true; // TODO: CHANGE FOR NEW ROBOT
    private static final boolean INVERT_LEFT_SIDE = false; // TODO: CHANGE FOR NEW ROBOT
    private static final boolean INVERT_RIGHT_SIDE = true; // TODO: CHANGE FOR NEW ROBOT

    private static final int kPigeonId = 40; // TODO: CHANGE FOR NEW ROBOT

    public static final SwerveDrivetrainConstants DrivetrainConstants =
        new SwerveDrivetrainConstants()
            .withCANBusName(CANBUS_NAME.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(PIGEON2_CONFIGS);

    // Uses SwerveModuleConstantsFactory to organize all the previously mentioned configurations
    // related to the Swerve Drive
    private static final SwerveModuleConstantsFactory<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        ConstantCreator =
            new SwerveModuleConstantsFactory<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                .withCouplingGearRatio(COUPLE_RATIO)
                .withWheelRadius(WHEEL_RADIUS_INCHES)
                .withSteerMotorGains(STEER_GAINS)
                .withDriveMotorGains(DRIVE_GAINS)
                .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
                .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
                .withSlipCurrent(SLIP_CURRENT_AMPS)
                .withSpeedAt12Volts(SPEED_AT_12V_METERS_PER_SECOND)
                .withDriveMotorType(DRIVE_MOTOR_TYPE)
                .withSteerMotorType(STEER_MOTOR_TYPE)
                .withFeedbackSource(STEER_FEEDBACK_TYPE)
                .withDriveMotorInitialConfigs(DRIVE_INITIAL_CONFIGS)
                .withSteerMotorInitialConfigs(STEER_INITIAL_CONFIGS)
                .withEncoderInitialConfigs(ENCODER_INITIAL_CONFIGS)
                .withSteerInertia(Simulation.STEER_INERTIA)
                .withDriveInertia(Simulation.DRIVE_INERTIA)
                .withSteerFrictionVoltage(Simulation.STEER_FRICTION_VOLTAGE)
                .withDriveFrictionVoltage(Simulation.DRIVE_FRICTION_VOLTAGE);

    // Front Left
    // TODO: CHANGE FOR NEW ROBOT
    private static final int FRONT_LEFT_STEER_MOTOR_ID = 3;
    private static final int FRONT_LEFT_DRIVE_MOTOR_ID = 4;
    private static final int FRONT_LEFT_ENCODER_ID = 21;
    private static final Angle FRONT_LEFT_ENCODER_OFFSET_ROT =
        WHICH_SWERVE_ROBOT.FRONT_LEFT_ENCODER_OFFSET;

    // TODO: CHANGE FOR NEW ROBOT
    private static final Distance FRONT_LEFT_X_POS =
        WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.length.div(2);
    private static final Distance FRONT_LEFT_Y_POS =
        WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.width.div(2);

    // Front Right
    // TODO: CHANGE FOR NEW ROBOT
    private static final int FRONT_RIGHT_STEER_MOTOR_ID = 5;
    private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 6;
    private static final int FRONT_RIGHT_ENCODER_ID = 22;
    private static final Angle FRONT_RIGHT_ENCODER_OFFSET_ROT =
        WHICH_SWERVE_ROBOT.FRONT_RIGHT_ENCODER_OFFSET;

    // TODO: CHANGE FOR NEW ROBOT
    private static final Distance FRONT_RIGHT_X_POS =
        WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.length.div(2);
    private static final Distance FRONT_RIGHT_Y_POS =
        WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.width.div(-2);

    // Back Left
    // TODO: CHANGE FOR NEW ROBOT
    private static final int BACK_LEFT_STEER_MOTOR_ID = 1;
    private static final int BACK_LEFT_DRIVE_MOTOR_ID = 2;
    private static final int BACK_LEFT_ENCODER_ID = 20;
    private static final Angle BACK_LEFT_ENCODER_OFFSET_ROT =
        WHICH_SWERVE_ROBOT.BACK_LEFT_ENCODER_OFFSET;

    // TODO: CHANGE FOR NEW ROBOT
    private static final Distance BACK_LEFT_X_POS =
        WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.length.div(-2);
    private static final Distance BACK_LEFT_Y_POS =
        WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.width.div(2);

    // Back Right
    // TODO: CHANGE FOR NEW ROBOT
    private static final int BACK_RIGHT_STEER_MOTOR_ID = 7;
    private static final int BACK_RIGHT_DRIVE_MOTOR_ID = 8;
    private static final int BACK_RIGHT_ENCODER_ID = 23;
    private static final Angle BACK_RIGHT_ENCODER_OFFSET_ROT =
        WHICH_SWERVE_ROBOT.BACK_RIGHT_ENCODER_OFFSET;

    // TODO: CHANGE FOR NEW ROBOT
    private static final Distance BACK_RIGHT_X_POS =
        WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.length.div(-2);
    private static final Distance BACK_RIGHT_Y_POS =
        WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.width.div(-2);

    // Set the constants per module (constants defined above)
    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        FrontLeft =
            ConstantCreator.createModuleConstants(
                FRONT_LEFT_STEER_MOTOR_ID,
                FRONT_LEFT_DRIVE_MOTOR_ID,
                FRONT_LEFT_ENCODER_ID,
                FRONT_LEFT_ENCODER_OFFSET_ROT,
                FRONT_LEFT_X_POS,
                FRONT_LEFT_Y_POS,
                INVERT_LEFT_SIDE,
                STEER_MOTOR_REVERSED,
                false);
    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        FrontRight =
            ConstantCreator.createModuleConstants(
                FRONT_RIGHT_STEER_MOTOR_ID,
                FRONT_RIGHT_DRIVE_MOTOR_ID,
                FRONT_RIGHT_ENCODER_ID,
                FRONT_RIGHT_ENCODER_OFFSET_ROT,
                FRONT_RIGHT_X_POS,
                FRONT_RIGHT_Y_POS,
                INVERT_RIGHT_SIDE,
                STEER_MOTOR_REVERSED,
                false);
    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        BackLeft =
            ConstantCreator.createModuleConstants(
                BACK_LEFT_STEER_MOTOR_ID,
                BACK_LEFT_DRIVE_MOTOR_ID,
                BACK_LEFT_ENCODER_ID,
                BACK_LEFT_ENCODER_OFFSET_ROT,
                BACK_LEFT_X_POS,
                BACK_LEFT_Y_POS,
                INVERT_LEFT_SIDE,
                STEER_MOTOR_REVERSED,
                false);
    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        BackRight =
            ConstantCreator.createModuleConstants(
                BACK_RIGHT_STEER_MOTOR_ID,
                BACK_RIGHT_DRIVE_MOTOR_ID,
                BACK_RIGHT_ENCODER_ID,
                BACK_RIGHT_ENCODER_OFFSET_ROT,
                BACK_RIGHT_X_POS,
                BACK_RIGHT_Y_POS,
                INVERT_RIGHT_SIDE,
                STEER_MOTOR_REVERSED,
                false);

    // These constants are necessary for new Telemetry with swerve
    // TODO: CHANGE FOR NEW ROBOT
    private double MAX_SPEED_MPS =
        SPEED_AT_12V_METERS_PER_SECOND.magnitude(); // kSpeedAt12Volts desired top speed
    private double MAX_ANGULAR_RATE_RPS =
        RotationsPerSecond.of(0.75)
            .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // TODO: CHANGE FOR NEW ROBOT
    // these outline the speed calculations
    public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 4.868;
    // 5.944; // before: 4.8768;// 18ft/s = 5.486, 19m/s = 5.791ft/s, 19.5m/s = 5.944 ft/s,
    public static final double PHYSICAL_MAX_ANGLUAR_SPEED_RADIANS_PER_SECOND = 10.917;
    public static final double TELE_DRIVE_FAST_MODE_SPEED_PERCENT = 0.5;
    public static final double TELE_DRIVE_SLOW_MODE_SPEED_PERCENT = 0.3;
    public static final double TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = 8;
    public static final double TELE_DRIVE_PERCENT_SPEED_RANGE =
        (TELE_DRIVE_FAST_MODE_SPEED_PERCENT - TELE_DRIVE_SLOW_MODE_SPEED_PERCENT);
    public static final double TELE_DRIVE_MAX_ANGULAR_RATE = 10.917;
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 26.971;
  }
}
