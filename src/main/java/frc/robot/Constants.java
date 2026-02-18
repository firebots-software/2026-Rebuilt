package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public final class Constants {
  public static final boolean hopperOnRobot = false;
  public static final boolean intakeOnRobot = false;
  public static final boolean visionOnRobot = false;
  public static final boolean shooterOnRobot = false;
  public static final boolean climberOnRobot = false;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class Simulation {
    public static final double SIM_LOOP_PERIOD_SECONDS =
        0.020; // time between updating the simulator
  }

  public static final class Intake {

    /** Constants for the intake deployment arm (four-bar linkage) */
    public static final class Arm {
      public static final double ARM_LENGTH_METERS = 0.35;

      public static final int CAN_ID = 14; // TODO: VERIFY
      public static final int ENCODER_PORT = 16; // TODO: VERIFY

      public static final double MOTOR_ROTS_PER_ARM_ROTS = (700.0 / 9.0);
      public static final double ARM_ROTS_PER_MOTOR_ROTS = 1.0 / MOTOR_ROTS_PER_ARM_ROTS;

      public static final double ARM_DEGREES_PER_MOTOR_ROTS = 360.0 / MOTOR_ROTS_PER_ARM_ROTS;
      // = 360 / 77.777 = 4.629 degrees per motor rotation

      public static final double MOTOR_ROTS_PER_ARM_DEGREES = MOTOR_ROTS_PER_ARM_ROTS / 360.0;

      // = 77.777 / 360 = 0.216 motor rotations per degree
      // public static final double MOTOR_ROTS_PER_ARM_DEGREES =
      //     Units.degreesToRotations(ARM_ROTS_PER_MOTOR_ROTS);

      // public static final double ARM_DEGREES_PER_MOTOR_ROTS = 1.0 / MOTOR_ROTS_PER_ARM_DEGREES;

      /** Absolute encoder ratio: 2.666:1 between encoder and axle */
      public static final double CANCODER_ROTS_PER_ARM_ROTS = (8.0 / 3.0);

      public static final double ARM_ROTS_PER_CANCODER_ROTS = 1.0 / CANCODER_ROTS_PER_ARM_ROTS;
      public static final double ENCODER_OFFSET = 0.0; // TODO: Calibrate on robot

      // Control Constants (Position closed-loop and torque control)
      // Note: MRD specifies <insert> for most values - these need characterization/tuning
      public static final double KV = 0.01; // V*s/rot - TODO: Verify on new robot
      public static final double KP = 80.0; // V/rot - TODO: Verify on new robot
      public static final double KI = 0.0;
      public static final double KD = 0.0; // V*s/rot - TODO: Verify on new robot
      public static final double KG = 0.15; // TODO: verify

      // Current Limits
      public static final double STATOR_CURRENT_LIMIT = 40.0; // Amps - TODO: Verify with team
      public static final double ARM_DEGREES_UPPER_LIMIT = 95.0;
      public static final double ARM_POS_RETRACTED = 90.0;
      public static final double ARM_POS_EXTENDED = 15.0;
      public static final double ARM_POS_MAX = 90.0;
      public static final double ARM_POS_MIN = 15.0;
      public static final double SIM_ARM_POS_MIN = 10.0; // for the simulator
      public static final double SIM_ARM_POS_MAX = 95.0;
      public static final double ARM_POS_IDLE = 45.0; // subject to change

      public static final double POSITION_TOLERANCE_DEGREES = 1.0;

      // Simulation
      public static final double SIM_MOI_KG_M2 = 0.1;
    }

    /** Constants for the intake roller wheels */
    public static final class Rollers {
      // Hardware Configuration
      public static final int CAN_ID = 11; // TODO: Get CAN ID from MRD table (currently blank)

      // Gear Ratios & Conversions
      /**
       * End-to-end reduction: 2.6667:1 Breakdown: Motor → 12t:32t pulley (9mm, 70t belt) → top
       * rollers → 17t:17t pulley (9mm, 65t belt) → bottom rollers
       */
      public static final double MOTOR_ROTS_PER_ROLLERS_ROTS = 8.0 / 3.0;

      public static final double ROLLER_ROTS_PER_MOTOR_ROTS = 1.0 / MOTOR_ROTS_PER_ROLLERS_ROTS;

      // Wheel Specifications
      /** Roller wheel diameter (inches) */
      public static final double ROLLER_DIAMETER_INCHES = 3.0;

      public static final double ROLLER_CIRCUMFERENCE_INCHES = ROLLER_DIAMETER_INCHES * Math.PI;

      /** Designed top speed: ~25 ft/s surface speed */
      public static final double DESIGNED_SURFACE_SPEED_FT_PER_SEC = 25.0;

      public static final double DESIGNED_SURFACE_SPEED_METERS_PER_SEC =
          Units.feetToMeters(DESIGNED_SURFACE_SPEED_FT_PER_SEC);
      public static final double DESIGNED_SURFACE_SPEED_IN_PER_SEC =
          DESIGNED_SURFACE_SPEED_FT_PER_SEC * 12.0;

      /** Target roller RPM to achieve designed surface speed */
      public static final double TARGET_ROLLER_RPM =
          (DESIGNED_SURFACE_SPEED_IN_PER_SEC * 60.0) / ROLLER_CIRCUMFERENCE_INCHES;

      /** Target roller RPS (rotations per second) */
      public static final double TARGET_ROLLER_RPS = TARGET_ROLLER_RPM / 60.0;

      /** Target motor velocity (RPS) to achieve designed roller speed */
      public static final double TARGET_MOTOR_RPS = TARGET_ROLLER_RPS * MOTOR_ROTS_PER_ROLLERS_ROTS;

      // Control Constants (Kraken x60, velocity closed-loop)
      public static final double KV = 0.14; // from MRDs
      public static final double KP = 0.0; // TODO: MRD shows <insert>
      public static final double KI = 0; // TODO: MRD shows <insert>
      public static final double KD = 0; // TODO: MRD shows <insert>

      // Current Limits
      public static final double STATOR_CURRENT_LIMIT = 80.0; // Amps - TODO: Verify with team
      public static final double SUPPLY_CURRENT_LIMIT = 80.0; // Amps - TODO: Verify with team

      // Ball Detection (monitors roller current to estimate ball intake)
      /** Current threshold indicating balls are being intaken */
      public static final double BALL_DETECTION_CURRENT_THRESHOLD_AMPS =
          15.0; // TODO: Tune empirically

      /** Debounce time for ball detection to filter noise */
      public static final double BALL_DETECTION_DEBOUNCE_SEC = 0.1; // TODO: Tune

      public static final double SIM_MOI_KG_M2 =
          0.0003; // TODO: BETTER ESTIMATION CAN BE MADE USING DESIGN

      public static final double TOLERANCE_MOTOR_ROTS_PER_SEC = 0.3; // TODO: OBSERVE BEHAVIOR
    }

    /**
     * Constants for power retract behavior during shooting. WARNING: Per MRD, only use after balls
     * have been partially emptied to avoid expelling balls from hopper.
     */
    public static final class PowerRetract {
      /**
       * Torque current for power retract mode (TorqueCurrentFOC). Applies constant force to help
       * push balls from hopper into shooter to increase BPS.
       */
      public static final double TORQUE_CURRENT_AMPS = 20.0; // TODO: Tune empirically
    }
  }

  public static class Swerve {
    public static final SwerveType WHICH_SWERVE_ROBOT = SwerveType.SERRANO;

    public static final double targetPositionError = 0.05;
    public static final double targetAngleError = 0.02;

    public static enum SwerveLevel {
      L2(6.75, 21.428571428571427),
      L3(6.12, 21.428571428571427),
      FIVEN_L3(5.2734375, 26.09090909091);
      public final double DRIVE_GEAR_RATIO, STEER_GEAR_RATIO;

      SwerveLevel(double drive, double steer) {
        DRIVE_GEAR_RATIO = drive;
        STEER_GEAR_RATIO = steer;
      }
    }

    public static enum SwerveDrivePIDValues {
      SERRANO(0.18014, 0d, 0d, -0.023265, 0.12681, 0.058864),
      PROTO(0.053218, 0d, 0d, 0.19977, 0.11198, 0.0048619),
      // JAMES_HARDEN(0.16901, 0d, 0d, 0.1593, 0.12143, 0.0091321); //0.041539
      // //0.12301
      JAMES_HARDEN(0.36, 0d, 0d, 0.2425, 0.11560693641, 0), // 0.041539 //0.12301
      COBRA(0.1, 0d, 0d, 0d, 0.124, 0d); // 0.041539 //0.12301
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
      JAMES_HARDEN(38.982d, 2.4768d, 0d, 0.23791d, 0d, 0.1151d),
      COBRA(100d, 0d, 0.5, 0.1, 2.49, 0d);
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

    public static enum ChoreoPIDValues {
      SERRANO(0.1d, 0d, 0d, 0.1d, 0d, 0d, 3.867d, 0d, 0d),
      PROTO(0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d),
      JAMES_HARDEN(0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d),
      COBRA(0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d);
      public final double kPX, kIX, kDX, kPY, kIY, kDY, kPR, kIR, kDR;

      ChoreoPIDValues(
          double kPX,
          double kIX,
          double kDX,
          double kPY,
          double kIY,
          double kDY,
          double kPR,
          double kIR,
          double kDR) {
        this.kPX = kPX;
        this.kIX = kIX;
        this.kDX = kDX;
        this.kPY = kPY;
        this.kIY = kIY;
        this.kDY = kDY;
        this.kPR = kPR;
        this.kIR = kIR;
        this.kDR = kDR;
      }
    }

    public static enum RobotDimensions {
      SERRANO(Inches.of(22.52), Inches.of(22.834)), // length, width
      PROTO(Inches.of(22.52), Inches.of(22.834)), // length, width
      JAMES_HARDEN(Inches.of(26.75), Inches.of(22.75)), // length, width
      COBRA(Inches.of(29.0), Inches.of(26.0)); // length, width
      public final Distance length, width;

      RobotDimensions(Distance length, Distance width) {
        this.length = length;
        this.width = width;
      }
    }

    public static enum BumperThickness {
      SERRANO(Inches.of(2.625)), // thickness
      PROTO(Inches.of(2.625)), // thickness
      JAMES_HARDEN(Inches.of(3.313)), // thickness
      COBRA(Inches.of(0)); // thickness

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
          ChoreoPIDValues.SERRANO,
          RobotDimensions.SERRANO,
          "Patrice the Pineapple",
          BumperThickness.SERRANO,
          3.5714285714285716,
          true),
      PROTO(
          Rotations.of(0.3876953125), // front left
          Rotations.of(0.159912109375), // front right
          Rotations.of(0.213134765625), // back left
          Rotations.of(-0.3818359375), // back right
          SwerveLevel.L2, // what level the swerve drive is
          SwerveDrivePIDValues.PROTO,
          SwerveSteerPIDValues.PROTO,
          ChoreoPIDValues.PROTO,
          RobotDimensions.PROTO,
          "rio",
          BumperThickness.PROTO,
          3.5714285714285716,
          true),
      JAMES_HARDEN(
          Rotations.of(-0.0834960938), // front left
          Rotations.of(-0.4912109375), // front right
          Rotations.of(0.1931152344), // back left
          Rotations.of(-0.15576171875), // back right
          SwerveLevel.L3,
          SwerveDrivePIDValues.JAMES_HARDEN,
          SwerveSteerPIDValues.JAMES_HARDEN,
          ChoreoPIDValues.JAMES_HARDEN,
          RobotDimensions.JAMES_HARDEN,
          "JamesHarden",
          BumperThickness.JAMES_HARDEN,
          3.5714285714285716,
          true),
      COBRA(
          Rotations.of(0.096923828125), // front left, 21
          Rotations.of(0.03271484375), // front right, 22
          Rotations.of(0.02587890625), // back left, 20
          Rotations.of(-0.09765625), // back right, 23
          SwerveLevel.FIVEN_L3,
          SwerveDrivePIDValues.COBRA,
          SwerveSteerPIDValues.COBRA,
          ChoreoPIDValues.COBRA,
          RobotDimensions.COBRA,
          "Viper",
          BumperThickness.COBRA,
          3.5714285714285716,
          false);
      public final Angle FRONT_LEFT_ENCODER_OFFSET,
          FRONT_RIGHT_ENCODER_OFFSET,
          BACK_LEFT_ENCODER_OFFSET,
          BACK_RIGHT_ENCODER_OFFSET;
      public final SwerveLevel SWERVE_LEVEL;
      public final SwerveDrivePIDValues SWERVE_DRIVE_PID_VALUES;
      public final SwerveSteerPIDValues SWERVE_STEER_PID_VALUES;
      public final ChoreoPIDValues CHOREO_PID_VALUES;
      public final RobotDimensions ROBOT_DIMENSIONS;
      public final String CANBUS_NAME;
      public final double COUPLE_RATIO;
      public final BumperThickness BUMPER_THICKNESS;
      public final boolean INVERTED_MODULES;

      SwerveType(
          Angle fl,
          Angle fr,
          Angle bl,
          Angle br,
          SwerveLevel swerveLevel,
          SwerveDrivePIDValues swerveDrivePIDValues,
          SwerveSteerPIDValues swerveSteerPIDValues,
          ChoreoPIDValues choreoPIDValues,
          RobotDimensions robotDimensions,
          String canbus_name,
          BumperThickness thickness,
          double coupled_ratio,
          boolean invertedModules) {
        FRONT_LEFT_ENCODER_OFFSET = fl;
        FRONT_RIGHT_ENCODER_OFFSET = fr;
        BACK_LEFT_ENCODER_OFFSET = bl;
        BACK_RIGHT_ENCODER_OFFSET = br;
        SWERVE_LEVEL = swerveLevel;
        SWERVE_DRIVE_PID_VALUES = swerveDrivePIDValues;
        SWERVE_STEER_PID_VALUES = swerveSteerPIDValues;
        CHOREO_PID_VALUES = choreoPIDValues;
        ROBOT_DIMENSIONS = robotDimensions;
        CANBUS_NAME = canbus_name;
        BUMPER_THICKNESS = thickness;
        COUPLE_RATIO = coupled_ratio;
        INVERTED_MODULES = invertedModules;
      }
    }

    // TODO: CHANGE FOR NEW ROBOT
    // these outline the speed calculations
    public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 4.868;
    // 5.944; // before: 4.8768;// 18ft/s = 5.486, 19m/s = 5.791ft/s, 19.5m/s = 5.944 ft/s,
    public static final double PHYSICAL_MAX_ANGLUAR_SPEED_RADIANS_PER_SECOND = 10.917;
    public static final double TELE_DRIVE_FAST_MODE_SPEED_PERCENT = 0.7;
    public static final double TELE_DRIVE_SLOW_MODE_SPEED_PERCENT = 0.3;
    public static final double TELE_DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_PER_SECOND = 8;
    public static final double TELE_DRIVE_PERCENT_SPEED_RANGE =
        (TELE_DRIVE_FAST_MODE_SPEED_PERCENT - TELE_DRIVE_SLOW_MODE_SPEED_PERCENT);
    public static final double TELE_DRIVE_MAX_ANGULAR_RATE_RADIANS_PER_SECOND = 10.917;
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_PER_SECOND =
        26.971;
  }

  public static class Climber {
    public static final double mmcV = 80; // TODO: acquire good ones
    public static final double mmcA = 80;

    public static final double KP = .4;
    public static final double KI = 0;
    public static final double KD = 0;

    public static final double DEFAULT_SUPPLY_CURRENT = 30.0;
    public static final double DEFAULT_STATOR_CURRENT = 30.0;

    public static final int BRAKE_PORT = 7; // TODO
    public static final double BRAKE_ANGLE = 30.0;

    public static class MuscleUp {
      public static final double MUSCLE_UP_TOLERANCE = 0.1;

      public static final double MOTOR_ROTS_TO_ARM_ROTS = 1d / 250d;
      public static final double MOTOR_ROTS_PER_DEGREES_OF_ARM_ROT = MOTOR_ROTS_TO_ARM_ROTS * 360d;

      // As I understand it, resting postion would probably always be consistent
      public static final double L1_MUSCLE_UP_FORWARD = 0; // TODO: get vals
      public static final double L2_MUSCLE_UP_FORWARD = 0; // TODO: get vals
      public static final double L3_MUSCLE_UP_FORWARD = 0; // TODO: get vals
      public static final double MUSCLE_UP_BACK = 0; // TODO: get vals

      public static final int MOTOR_PORT = -1; // TODO: get vals

      public static final int ENCODER_PORT = -1; // TODO: get vals
      public static final int ENCODER_ROTATIONS_TO_ARM_ROTATIONS = 0;
      public static final int ENCODER_OFFSET = 0; // TODO: get vals
    }

    public static class SitUp {
      public static final double SIT_UP_TOLERANCE = .1;

      public static final double MOTOR_ROTS_TO_ARM_ROTS = 1d / 100d;
      public static final double MOTOR_ROTS_PER_DEGREES_OF_ARM_ROT = MOTOR_ROTS_TO_ARM_ROTS * 360d;

      public static final double CURRENT_SUPPLY_LIMIT = 60;
      public static final double CURRENT_STATOR_LIMIT = 100;

      public static final double SIT_UP_ANGLE = 0; // TODO: get vals
      public static final double SIT_BACK_ANGLE = 0; // TODO: get vals

      public static final int MOTOR_PORT = -1; // TODO: get vals

      public static final int ENCODER_PORT = -1; // TODO: get vals
      public static final int ENCODER_ROTATIONS_TO_ARM_ROTATIONS = 0;
      public static final int ENCODER_OFFSET = 0; // TODO: get vals
    }

    public static class PullUp {
      public static final double PULL_UP_TOLERANCE = .1;

      public static final double MOTOR_ROTS_TO_PULLEY_ROTS = 1d / 17d;
      public static final double PULLEY_BELT_LENGTH_M = 0; // TODO: get actual value
      public static final double MOTOR_ROTS_PER_METERS_OF_BELT_TRAVERSAL =
          MOTOR_ROTS_TO_PULLEY_ROTS * PULLEY_BELT_LENGTH_M;

      // As I understand it, resting postion would probably always be consistent
      public static final double L1_REACH_POS = 0; // TODO: get vals
      public static final double L2_REACH_POS = 0; // TODO: get vals
      public static final double L3_REACH_POS = 0; // TODO: get vals
      public static final double PULL_DOWN_POS = 0; // TODO: get vals

      public static final int MOTOR_PORT_L = -1; // TODO: get vals
      public static final int MOTOR_PORT_R = -1; // TODO: get vals
    }
  }

  public static class Hopper {
    // --- Mechanical transmission ---
    // Motor turns needed for one hopper pulley turn (5:1 reduction)
    public static final double MOTOR_ROTATIONS_PER_HOPPER_PULLEY_ROTATION = 5.0;
    public static final double MOTOR_ROTATIONS_PER_AGITATOR_ROTATION =
        (20.0 / 24.0) * (60.0 / 12.0);

    // Timing belt geometry
    public static final double HOPPER_BELT_TOOTH_PITCH_METERS =
        0.005; // length of belt movement per tooth moved on it
    public static final double HOPPER_BELT_TOOTH_COUNT =
        220.0; // number of teeth on the actual belt for full revolution
    public static final double HOPPER_BELT_LOOP_LENGTH_METERS =
        HOPPER_BELT_TOOTH_COUNT * HOPPER_BELT_TOOTH_PITCH_METERS; // total length of the belt

    // Linear travel conversion
    // meters of belt travel per motor rotation
    public static final double HOPPER_BELT_METERS_PER_MOTOR_ROTATION =
        HOPPER_BELT_LOOP_LENGTH_METERS / MOTOR_ROTATIONS_PER_HOPPER_PULLEY_ROTATION;

    // inverse conversion (sometimes convenient in control code)
    // motor rotations per meter of belt travel
    public static final double MOTOR_ROTATIONS_PER_HOPPER_BELT_METER =
        1.0 / HOPPER_BELT_METERS_PER_MOTOR_ROTATION;

    public static final double AGITATOR_ROTATIONS_PER_MOTOR_ROTATION =
        1.0 / MOTOR_ROTATIONS_PER_AGITATOR_ROTATION;

    // --- Operating targets ---
    public static final double HOPPER_BELT_TARGET_SPEED_FEET_PER_SECOND = 6.0;
    public static final double HOPPER_BELT_TARGET_SPEED_METERS_PER_SECOND =
        Units.feetToMeters(HOPPER_BELT_TARGET_SPEED_FEET_PER_SECOND);

    // --- Hardware IDs ---
    public static final int MOTOR_PORT = 9;

    // --- Closed-loop velocity gains (Phoenix Slot0) ---
    public static final double kP = 0.01;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.124;

    // --- Current limits ---
    public static final double HOPPER_STATOR_LIMIT_AMPS = 30.0;
    public static final double HOPPER_SUPPLY_LIMIT_AMPS = 30.0;

    // --- Control tolerance ---
    public static final double HOPPER_VELOCITY_TOLERANCE_ROTATIONS_PER_SECOND = 0.1;

    // --- Simulation ---
    public static final double HOPPER_SIM_MECHANISM_MOI_KG_M2 = 0.0008;
  }

  public static class Vision {

    // initializes cameras for use in VisionSubsystem
    public static enum Cameras {
      RIGHT_CAM("frontRightCam"),
      LEFT_CAM("frontLeftCam"),
      REAR_RIGHT_CAM("rearRightCam"),
      REAR_LEFT_CAM("rearLeftCam"),
      COLOR_CAM("colorCam");

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

    // TODO: SID: update all vals
    public static final double FRONT_RIGHT_X = Units.inchesToMeters(6.70);
    public static final double FRONT_RIGHT_Y = Units.inchesToMeters(-4.125);
    public static final double FRONT_RIGHT_Z = Units.inchesToMeters(40.875);
    public static final double FRONT_RIGHT_ROLL = Units.degreesToRadians(180); // 180
    public static final double FRONT_RIGHT_PITCH = Units.degreesToRadians(171.5); // 171.5
    public static final double FRONT_RIGHT_YAW = Units.degreesToRadians(0.0);

    public static final double FRONT_LEFT_X = Units.inchesToMeters(6.70);
    public static final double FRONT_LEFT_Y = Units.inchesToMeters(4.125);
    public static final double FRONT_LEFT_Z = Units.inchesToMeters(40.875);
    public static final double FRONT_LEFT_ROLL = Units.degreesToRadians(180);
    public static final double FRONT_LEFT_PITCH = Units.degreesToRadians(171.5);
    public static final double FRONT_LEFT_YAW = Units.degreesToRadians(0.0);

    public static final double REAR_RIGHT_X = Units.inchesToMeters(6.70);
    public static final double REAR_RIGHT_Y = Units.inchesToMeters(-4.125);
    public static final double REAR_RIGHT_Z = Units.inchesToMeters(40.875);
    public static final double REAR_RIGHT_ROLL = Units.degreesToRadians(180); // 180
    public static final double REAR_RIGHT_PITCH = Units.degreesToRadians(171.5); // 171.5
    public static final double REAR_RIGHT_YAW = Units.degreesToRadians(0.0);

    public static final double REAR_LEFT_X = Units.inchesToMeters(6.70);
    public static final double REAR_LEFT_Y = Units.inchesToMeters(4.125);
    public static final double REAR_LEFT_Z = Units.inchesToMeters(40.875);
    public static final double REAR_LEFT_ROLL = Units.degreesToRadians(180);
    public static final double REAR_LEFT_PITCH = Units.degreesToRadians(171.5);
    public static final double REAR_LEFT_YAW = Units.degreesToRadians(0.0);

    public static final double COLOR_X = Units.inchesToMeters(8.867);
    public static final double COLOR_Y = Units.inchesToMeters(12.478);
    public static final double COLOR_Z = Units.inchesToMeters(6.158);
    public static final double COLOR_ROLL = Units.degreesToRadians(0.0);
    public static final double COLOR_PITCH = Units.degreesToRadians(8.7);
    public static final double COLOR_YAW = Units.degreesToRadians(0.0);

    // initializing Transform3d for use in future field visualization
    public static Transform3d getCameraTransform(Cameras camera) {
      switch (camera) {
        case RIGHT_CAM: // TODO: SID: rename FRONT_RIGHT_CAM
          return new Transform3d(
              new Translation3d(FRONT_RIGHT_X, FRONT_RIGHT_Y, FRONT_RIGHT_Z),
              new Rotation3d(FRONT_RIGHT_ROLL, FRONT_RIGHT_PITCH, FRONT_RIGHT_YAW));

        case LEFT_CAM: // TODO: SID: rename FRONT_LEFT_CAM
          return new Transform3d(
              new Translation3d(FRONT_LEFT_X, FRONT_LEFT_Y, FRONT_LEFT_Z),
              new Rotation3d(FRONT_LEFT_ROLL, FRONT_LEFT_PITCH, FRONT_LEFT_YAW));

        case REAR_RIGHT_CAM:
          return new Transform3d(
              new Translation3d(REAR_RIGHT_X, REAR_RIGHT_Y, REAR_RIGHT_Z),
              new Rotation3d(REAR_RIGHT_ROLL, REAR_RIGHT_PITCH, REAR_RIGHT_YAW));

        case REAR_LEFT_CAM:
          return new Transform3d(
              new Translation3d(REAR_LEFT_X, REAR_LEFT_Y, REAR_LEFT_Z),
              new Rotation3d(REAR_LEFT_ROLL, REAR_LEFT_PITCH, REAR_LEFT_YAW));

        case COLOR_CAM:
          return new Transform3d(
              new Translation3d(COLOR_X, COLOR_Y, COLOR_Z),
              new Rotation3d(COLOR_ROLL, COLOR_PITCH, COLOR_YAW));
        default:
          throw new IllegalArgumentException("Unknown camera ID: " + camera);
      }
    }
  }

  public static class FuelGaugeDetection {
    public static final int BALLS_TO_AVG = 3;
    public static final int MAX_FUEL_GAUGE_MEASUREMENTS = 33;
    public static final double MAX_DETECTABLE_FUEL_AREA_PERCENTAGE = 60.00;
    public static final double REALISTIC_MAX_DETECTABLE_AREA_PERCENTAGE = 15.00;

    public static enum FuelGauge { // LAST: 20, 50, 70, 100
      EMPTY(2.0),
      LOW(9.0),
      MEDIUM(12.0),
      FULL(100.0);

      private double threshold;

      FuelGauge(double threshold) {
        this.threshold = threshold;
      }

      public double getThreshold() {
        return threshold;
      }
    }
  }

  public static final class Shooter {
    public static final int WARMUP_1_ID = 35; // TODO
    public static final int WARMUP_2_ID = 34; // TODO
    public static final int WARMUP_3_ID = 32; // TODO

    public static final double SHOOTER_KP = 0.5; // TODO
    public static final double SHOOTER_KI = 0.0; // TODO
    public static final double SHOOTER_KD = 0.0; // TODO
    public static final double SHOOTER_KV = 0.12; // TODO
    public static final double SHOOTER_KA = 0.0; // TODO
    public static final double STATOR_CURRENT_LIMIT = 30.0;
    public static final double SUPPLY_CURRENT_LIMIT = 30.0;

    public static final double MOTOR_ROTS_PER_WHEEL_ROTS = 1.25;
    public static final double SHOOTER_WHEEL_DIAMETER = 3.0;
    public static final double SHOOT_FOR_AUTO = 104.72;

    public static final Pose3d OFFSET_FROM_ROBOT_CENTER = new Pose3d();

    public static final double SHOOTER_ANGLE_FROM_HORIZONTAL_DEGREES = 75;

    public static final boolean SHOOTS_BACKWARDS = false;

    public static final double ANGULAR_TOLERANCE_FOR_AUTO_AIM_RAD = .1;

    public static final int TARGETING_CALCULATION_PRECISION = 5;

    public static final double SHOOTER_SIM_MOI_KG_M2 = 0.0015;
  }

  public static class OI {
    public static final double LEFT_JOYSTICK_DEADBAND = 0.07;
    public static final double RIGHT_JOYSTICK_DEADBAND = 0.07;
    public static final int JOYSTICK_A_PORT = 0;

    public enum XBoxButtonID {
      A(1),
      B(2),
      X(3),
      Y(4),
      LeftBumper(5),
      RightBumper(6),
      LeftStick(9),
      RightStick(10),
      Back(7),
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

  public static class Landmarks {
    public static Pose3d BLUE_HUB =
        new Pose3d(4.621390342712402, 4.032095909118652, 0, new Rotation3d());
    public static Pose3d RED_HUB =
        new Pose3d(11.917659759521484, 4.032095909118652, 0, new Rotation3d());
  }
}
