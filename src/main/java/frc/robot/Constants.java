// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants.Swerve.BumperThickness;
import frc.robot.Constants.Swerve.RobotDimensions;
import frc.robot.Constants.Swerve.SwerveDrivePIDValues;
import frc.robot.Constants.Swerve.SwerveLevel;
import frc.robot.Constants.Swerve.SwerveSteerPIDValues;
import frc.robot.Constants.Swerve.SwerveType;

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

  public static class Landmarks {
    public static Pose3d BLUE_HUB = new Pose3d(4.621390342712402, 4.032095909118652, 0, new Rotation3d());
    public static Pose3d RED_HUB = new Pose3d(11.917659759521484, 4.032095909118652, 0, new Rotation3d());
  }

  public static class MotorConstants {
    public int port;

    public MotorConstants(int port) {
      this.port = port;
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
    public static final double TELE_DRIVE_FAST_MODE_SPEED_PERCENT = 0.7;
    public static final double TELE_DRIVE_SLOW_MODE_SPEED_PERCENT = 0.3;
    public static final double TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = 8;
    public static final double TELE_DRIVE_PERCENT_SPEED_RANGE =
        (TELE_DRIVE_FAST_MODE_SPEED_PERCENT - TELE_DRIVE_SLOW_MODE_SPEED_PERCENT);
    public static final double TELE_DRIVE_MAX_ANGULAR_RATE = 10.917;
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 26.971;
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

  public static class Kalman {
    public static final Matrix<N3, N1> visionMatrix = VecBuilder.fill(0.01, 0.03d, 100d);
    public static final Matrix<N3, N1> odometryMatrix = VecBuilder.fill(0.1, 0.1, 0.1);
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
}
