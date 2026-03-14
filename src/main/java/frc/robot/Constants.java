package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public final class Constants {
  public static final boolean hopperOnRobot = true;
  public static final boolean intakeOnRobot = true;
  public static final boolean visionOnRobot = true;
  public static final boolean fuelGaugeOnRobot = true;
  public static final boolean shooterOnRobot = true;
  public static final boolean climberOnRobot = true;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class Simulation {
    public static final double SIM_LOOP_PERIOD_SECONDS =
        0.020; // time between updating the simulator
  }

  public static final class Intake {
    public static final double HAS_PIECE_CURRENT_AMPS = 35.0;
    public static final double HAS_PIECE_THRESHOLD_SEC = 0.15;

    public static final class Arm {
      public static final int CAN_ID = 14;
      public static final int ENCODER_PORT = 15;

      // Current Limits
      public static final double ARM_POS_RETRACTED = 126.56;
      public static final double ARM_POS_EXTENDED = 29.85;
      public static final double ARM_POS_MAX = ARM_POS_RETRACTED;
      public static final double ARM_POS_MIN = 19.7;
      public static final double ARM_POS_IDLE = 73.7;

      public static final double POSITION_TOLERANCE_DEGREES = 1.0;

      public static final double POWER_RETRACT_TORQUE_CURRENT = 60.0; // TODO: Tune empirically
      public static final double POWER_RETRACT_DELAY = 0.2;

      // TODO: Tune
      public static final double kV = 0.124;
      public static final double kP = 63.0;
      public static final double kI = 0.1;
      public static final double kD = 0.0;
      public static final double kG = 1.15; // 0.69 recalc
      public static final double kS = 0.4;

      public static final double mmcV = 6.0;
      public static final double mmcA = 14.0;

      public static final double STATOR_CURRENT_LIMIT = 80.0; // TODO: Verify
      public static final double SUPPLY_CURRENT_LIMIT = 60.0; // TODO: Verify

      public static final double MOTOR_ROTS_PER_ARM_ROT =
          (12.0 / 1.0) * (42.0 / 36.0) * (30.0 / 18.0) * (32.0 / 20.0);
      public static final double ARM_ROTS_PER_MOTOR_ROT = 1.0 / MOTOR_ROTS_PER_ARM_ROT;
      public static final double ARM_DEGREES_PER_MOTOR_ROT = 360.0 / MOTOR_ROTS_PER_ARM_ROT;
      public static final double MOTOR_ROTS_PER_ARM_DEGREE = MOTOR_ROTS_PER_ARM_ROT / 360.0;
      public static final double CANCODER_ROTS_PER_ARM_ROT = (8.0 / 3.0);
      public static final double ARM_ROTS_PER_CANCODER_ROT = 1.0 / CANCODER_ROTS_PER_ARM_ROT;
      public static final double ENCODER_OFFSET = 0.06;

      public static final class Simulation {
        public static final double SIM_ARM_POS_MIN = 10.0;
        public static final double SIM_ARM_POS_MAX = 95.0;
        public static final double SIM_MOI_KG_M2 = 0.1;
        public static final double ARM_LENGTH_METERS = 0.35;
      }
    }

    public static final class Rollers {
      // Hardware Configuration
      public static final int CAN_ID = 16; // TODO: Get CAN ID from MRD table (currently blank)

      public static final double TOLERANCE_MOTOR_ROTS_PER_SEC = 2.0; // TODO: Tune

      // TODO: Tune
      public static final double kV = 0.1167;
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;

      // Current Limits
      public static final double STATOR_CURRENT_LIMIT = 80.0; // TODO: Verify
      public static final double SUPPLY_CURRENT_LIMIT = 80.0; // TODO: Verify

      public static final double ROLLER_CIRCUMFERENCE_INCHES = 3.0 * Math.PI;
      public static final double MOTOR_ROTS_PER_ROLLERS_ROT = 8.0 / 3.0;
      public static final double ROLLER_ROTS_PER_MOTOR_ROT = 1.0 / MOTOR_ROTS_PER_ROLLERS_ROT;
      public static final double DESIGNED_SURFACE_SPEED_FT_PER_SEC = 25.0;
      public static final double DESIGNED_SURFACE_SPEED_METERS_PER_SEC =
          DESIGNED_SURFACE_SPEED_FT_PER_SEC * 0.3048;
      public static final double DESIGNED_SURFACE_SPEED_IN_PER_SEC =
          DESIGNED_SURFACE_SPEED_FT_PER_SEC * 12.0;

      // public static final double TARGET_ROLLER_RPM =
      //     (DESIGNED_SURFACE_SPEED_IN_PER_SEC * 60.0) / ROLLER_CIRCUMFERENCE_INCHES;
      public static final double TARGET_ROLLER_RPM =
          2100;
      public static final double TARGET_ROLLER_RPS = TARGET_ROLLER_RPM / 60.0;
      public static final double TARGET_MOTOR_RPS = TARGET_ROLLER_RPS * MOTOR_ROTS_PER_ROLLERS_ROT;

      public static final class Simulation {
        public static final double SIM_MOI_KG_M2 = 0.0003;
      }
    }
  }

  public static class Swerve {
    public static final double FF_RADIUS_M2 = 0.1;
    public static final SwerveType WHICH_SWERVE_ROBOT = SwerveType.COBRA;
    public static final CANBus CAN_BUS =
        new CANBus(WHICH_SWERVE_ROBOT.CANBUS_NAME, "./logs/example.hoot");
    // the distance over the bump in meters

    public static final double targetPositionError = 0.03;
    public static final double targetAngleError = 0.1;
    public static final double MAX_HEADING_TRACKING_ROT_RATE_RADS_PER_SECOND = 4;

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

    public static enum SwerveDriveToPosePIDValues {
      SERRANO(3.067, 0, 0, 4.167, 0, 0, 3.667, 0, 0),
      PROTO(0, 0, 0, 0, 0, 0, 0, 0, 0),
      JAMES_HARDEN(0, 0, 0, 0, 0, 0, 0, 0, 0),
      COBRA(3.467, 0, 0, 3.567, 0, 0, 2.867, 0, 0);

      public final double kPX;
      public final double kIX;
      public final double kDX;
      public final double kPY;
      public final double kIY;
      public final double kDY;
      public final double kPR;
      public final double kIR;
      public final double kDR;

      SwerveDriveToPosePIDValues(
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

    public static enum SwerveDriveToPoseProfileValues {
      SERRANO(2, 2, 2, 2),
      PROTO(0.5, 0.5, 0.2, 0.2),
      JAMES_HARDEN(0.5, 0.5, 0.2, 0.2),
      COBRA(0.5, 0.5, 0.5, 0.5); // 5.67, 8.67, 1.9, 1.9
      public final double maxVelocityLinear,
          maxAccelerationLinear,
          maxVelocityAngular,
          maxAccelerationAngular;

      SwerveDriveToPoseProfileValues(
          double maxVelocityLinear,
          double maxAccelerationLinear,
          double maxVelocityAngular,
          double maxAccelerationAngular) {
        this.maxVelocityLinear = maxVelocityLinear;
        this.maxAccelerationLinear = maxAccelerationLinear;
        this.maxVelocityAngular = maxVelocityAngular;
        this.maxAccelerationAngular = maxAccelerationAngular;
      }
    }

    public static enum ChoreoPIDValues {
      SERRANO(0.1d, 0d, 0d, 0.1d, 0d, 0d, 3.867d, 0d, 0d),
      PROTO(0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d),
      JAMES_HARDEN(0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d),
      COBRA(3.467, 0, 0, 3.567, 0, 0, 2.867, 0, 0);
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
          SwerveDriveToPosePIDValues.SERRANO,
          SwerveDriveToPoseProfileValues.SERRANO,
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
          SwerveDriveToPosePIDValues.PROTO,
          SwerveDriveToPoseProfileValues.PROTO,
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
          SwerveDriveToPosePIDValues.JAMES_HARDEN,
          SwerveDriveToPoseProfileValues.JAMES_HARDEN,
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
          SwerveDriveToPosePIDValues.COBRA,
          SwerveDriveToPoseProfileValues.COBRA,
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
      public final SwerveDriveToPosePIDValues SWERVE_DRIVE_TO_POSE_PID_VALUES;
      public final SwerveDriveToPoseProfileValues SWERVE_DRIVE_TO_POSE_PROFILE_VALUES;
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
          SwerveDriveToPosePIDValues swerveDriveToPosePIDValues,
          SwerveDriveToPoseProfileValues swerveDriveToPoseProfileValues,
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
        SWERVE_DRIVE_TO_POSE_PID_VALUES = swerveDriveToPosePIDValues;
        SWERVE_DRIVE_TO_POSE_PROFILE_VALUES = swerveDriveToPoseProfileValues;
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
    // 5.944; // before: 4.8768;// 18ft/s = 5.486, 19m/s = 5.791ft/s, 19.5m/s =
    // 5.944 ft/s,
    public static final double PHYSICAL_MAX_ANGLUAR_SPEED_RADIANS_PER_SECOND = 10.917;
    public static final double TELE_DRIVE_FAST_MODE_SPEED_PERCENT = 0.7;
    public static final double TELE_DRIVE_SLOW_MODE_SPEED_PERCENT = 0.3;
    public static final double TELE_DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_PER_SECOND = 8;
    public static final double TELE_DRIVE_PERCENT_SPEED_RANGE =
        (TELE_DRIVE_FAST_MODE_SPEED_PERCENT - TELE_DRIVE_SLOW_MODE_SPEED_PERCENT);
    public static final double TELE_DRIVE_MAX_ANGULAR_RATE_RADIANS_PER_SECOND = 10.917;
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_PER_SECOND =
        26.971;

    public static class Auto {
      public static final double TIME_FOR_OUTPOST_INTAKE = 3.0;
      public static final double TIME_FOR_BUMP_FORWARDS = 0.95;
      public static final double TIME_FOR_BUMP_BACKWARDS = 0.70;

      public static enum Intake {
        LeftIntakeSweep,
        RightIntakeSweep,
        LeftIntakeSweepShort,
        RightIntakeSweepShort,
        p2IntakeForwardRight,
        p2IntakeForwardLeftShort,
        p2IntakeSideLeftShort,
        p2IntakeSideLeftShortHook
      }

      public static enum ShootPos {
        LeftShoot,
        RightShoot,
        DepotToShoot,
        OutpostToShoot,
        LeftShootSide
      }

      public static enum ClimbPos {
        LeftClimbL,
        LeftClimbR,
        RightClimbL,
        RightClimbR
      }

      public static enum Depot {
        DepotL,
        DepotR,
        DepotStart
      }

      public static enum Outpost {
        OutpostL,
        OutpostR,
        OutpostStart
      }

      public static enum MiscPaths {
        LeftSweep,
        RightSweep,
        LeftShootToBump,
        RightShootToBump,
        Nike
      }
    }
  }

  public static class Climber {
    public static final double DEFAULT_SUPPLY_CURRENT = 20.0; // 12.0
    public static final double DEFAULT_STATOR_CURRENT = 12.0; // 6.0

    public static final int BRAKE_PORT = 7; // TODO
    public static final double BRAKE_ANGLE = 24.838;
    public static final double UNBRAKE_ANGLE = 0.0;

    public static class MuscleUp {
      public static final int MOTOR_PORT = 11;

      public static final double KP = 0.4;
      public static final double KI = 0;
      public static final double KD = 0;
      public static final double KV = 0.15;
      public static final double KG = 0.3;
      public static final double KS = 0;

      public static final double MUSCLE_UP_TOLERANCE = 0.1;

      public static final double MOTOR_ROTS_PER_ARM_ROTS =
          (1.0 / 20.0) * (16.0 / 46.0) * (24.0 / 52.0) * (1.0 / 2.0);
      public static final double ARM_ROTS_PER_MOTOR_ROTS = 1.0 / MOTOR_ROTS_PER_ARM_ROTS;
      public static final double MOTOR_ROTS_PER_ARM_DEGREES = ARM_ROTS_PER_MOTOR_ROTS / 360d;
      public static final double ARM_DEGREES_PER_MOTOR_ROTS = 1 / MOTOR_ROTS_PER_ARM_DEGREES;

      // As I understand it, resting postion would probably always be consistent
      public static final double L1_MUSCLE_UP_FORWARD =
          95; // TODO: get vals, true val is 96.927 for all, 95 for
      // testing
      public static final double L2_MUSCLE_UP_FORWARD = 95; // TODO: get vals
      public static final double L3_MUSCLE_UP_FORWARD = 95; // TODO: get vals
      public static final double MUSCLE_UP_BACK = 0;
      public static final double MUSCLEUP_DOWN_VELOCITY = -0.2;

      public static final double CLIMBING_STATOR_CURRENT_LIMIT = 20.0;
      public static final double CLIMBING_SUPPLY_CURRENT_LIMIT = 40.0;

      public static final double ZEROING_STATOR_CURRENT_LIMIT = 7.0;
      public static final double ZEROING_SUPPLY_CURRENT_LIMIT = 11.0;
    }

    public static class SitUp {
      public static final int MOTOR_PORT = 12;
      public static final int ENCODER_PORT = 13;

      public static final double ENCODER_OFFSET = -0.1; // TODO: get vals

      public static final double KP = 500.0;
      public static final double KI = 0;
      public static final double KD = 0;
      public static final double KV = 0.12;
      public static final double KG = 0;
      public static final double KS = 0;

      public static final double SIT_UP_TOLERANCE = 0.1;

      public static final double MOTOR_ROTS_PER_ENCODER_ROT = 54.4;
      public static final double ENCODER_ROTS_PER_MOTOR_ROT = 1.0 / MOTOR_ROTS_PER_ENCODER_ROT;

      public static final double ENCODER_ROTS_PER_ARM_ROT = 32f / 17f;
      public static final double ARM_ROTS_PER_ENCODER_ROT = 1.0 / ENCODER_ROTS_PER_ARM_ROT;

      public static final double MOTOR_ROTS_PER_ARM_ROT =
          (1.0 / ENCODER_ROTS_PER_MOTOR_ROT) * (1.0 / ARM_ROTS_PER_ENCODER_ROT);
      public static final double ARM_ROTS_PER_MOTOR_ROT = 1.0 / MOTOR_ROTS_PER_ARM_ROT;

      public static final double SIT_UP_ANGLE_DEGREES = 57.0;
      public static final double SIT_BACK_ANGLE_DEGREES = 80.0;

      public static final double STATOR_CURRENT_LIMIT = 20.0;
      public static final double SUPPLY_CURRENT_LIMIT = 30.0;
    }

    public static class PullUp {
      public static final int MOTOR_L_PORT = 10;
      public static final int MOTOR_R_PORT = 9;

      public static final double KP = 2.0;
      public static final double KI = 0;
      public static final double KD = 0;
      public static final double KV = 0.124;
      public static final double KG = 0;
      public static final double KS = 0;

      public static final double PULL_UP_TOLERANCE_METERS = 0.1;

      public static final double MOTOR_ROTS_PER_BELT_METERS = 151.875;

      public static final double L1_REACH_POS = 0;
      public static final double L2_REACH_POS = 0;
      public static final double L3_REACH_POS = 0;
      public static final double PULL_DOWN_POS = 0;
      public static final double PULL_DOWN_POS_L1_AUTO = -0.192885 + 0.369885;
      public static final double PULL_DOWN_POS_METERS = 0.369885;

      public static final double CLIMBING_STATOR_CURRENT_LIMIT = 40.0;
      public static final double CLIMBING_SUPPLY_CURRENT_LIMIT = 60.0;

      public static final double ZEROING_STATOR_CURRENT_LIMIT = 15.0;
      public static final double ZEROING_SUPPLY_CURRENT_LIMIT = 15.0;

      public static final double PULL_DOWN_VELOCITY = -1.0;
      public static final double PULL_UP_VELOCITY = 5.0;
    }
  }

  public static class Hopper {
    public static final int MOTOR_PORT = 17;

    public static final double TARGET_SURFACE_SPEED_FPS = 6.0;
    public static final double TARGET_SURFACE_SPEED_MPS = TARGET_SURFACE_SPEED_FPS * 0.3048;

    public static final double FLOOR_SPEED_TOLERANCE_MPS = 0.05;

    // TODO: Tune
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.124;

    public static final double STATOR_LIMIT_AMPS = 150.0; // 50.0
    public static final double SUPPLY_LIMIT_AMPS = 50.0; // 30.0

    public static final double MOTOR_ROTATIONS_PER_FLOOR_PULLEY_ROTATION = 5.0;
    public static final double MOTOR_ROTATIONS_PER_AGITATOR_ROTATION =
        (20.0 / 24.0) * (60.0 / 12.0);

    public static final double BELT_TOOTH_PITCH_METERS =
        0.005; // length of belt movement per tooth moved on it
    public static final double FLOOR_PULLEY_TOOTH_COUNT = 24.0;
    public static final double BELT_LOOP_TOOTH_COUNT =
        220.0; // number of teeth on the actual belt for full revolution
    public static final double BELT_LOOP_LENGTH_METERS =
        BELT_LOOP_TOOTH_COUNT * BELT_TOOTH_PITCH_METERS;

    public static final double BELT_TRAVEL_METERS_PER_PULLEY_ROTATION =
        FLOOR_PULLEY_TOOTH_COUNT * BELT_TOOTH_PITCH_METERS;

    public static final double BELT_TRAVEL_METERS_PER_MOTOR_ROTATION =
        BELT_TRAVEL_METERS_PER_PULLEY_ROTATION / MOTOR_ROTATIONS_PER_FLOOR_PULLEY_ROTATION;

    public static final double MOTOR_ROTATIONS_PER_BELT_TRAVEL_METER =
        1.0 / BELT_TRAVEL_METERS_PER_MOTOR_ROTATION;

    public static final double AGITATOR_ROTATIONS_PER_MOTOR_ROTATION =
        1.0 / MOTOR_ROTATIONS_PER_AGITATOR_ROTATION;

    public static final InterpolatingDoubleTreeMap HOPPER_FPS_FOR_SHOOTER_WHEEL_RPS =
        new InterpolatingDoubleTreeMap();

    static {
      UPDATE_INTERMAPS();
    }

    public static void UPDATE_INTERMAPS() {
      HOPPER_FPS_FOR_SHOOTER_WHEEL_RPS.clear();

      HOPPER_FPS_FOR_SHOOTER_WHEEL_RPS.put(45.0, 2.2);
      HOPPER_FPS_FOR_SHOOTER_WHEEL_RPS.put(53.0, 2.2);
      HOPPER_FPS_FOR_SHOOTER_WHEEL_RPS.put(62.0, 0.88);
    }

    public static class Simulation {
      public static final double MECHANISM_SIM_MOI_KG_M2 = 0.0008;
    }
  }

  public static class Vision {

    // TODO: be able to set this at the start of the match
    public static VisionCamera FALLBACK_CAMERA = VisionCamera.FRONT_LEFT_CAM;
    public static boolean SKIP_TO_FALLBACK = false;

    // TODO: move this somewhere else
    public static void updateFallbackCamera(VisionCamera cam) {
      FALLBACK_CAMERA = cam;
    }

    public static final double MAX_TAG_DISTANCE = 15.0; // meters, beyond which readings are dropped

    public static final double CALIBRATION_FACTOR = 1.0;
    public static final double BASE_NOISE_X = 0.0008; // m
    public static final double BASE_NOISE_Y = 0.0008; // m
    public static final double BASE_NOISE_THETA = 0.5; // rad

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

    public static final double TIMESTAMP_THRESHOLD = 0.5;
    public static final double TIMESTAMP_FPGA_CORRECTION = -0.03;

    // TODO: SID: update all vals

    private class FrontRight {
      private static final double X = Units.inchesToMeters(-4.775894);
      private static final double Y = Units.inchesToMeters(-7.880312);
      private static final double Z = Units.inchesToMeters(27.024842);
      private static final double ROLL = Units.degreesToRadians(358.781735);
      private static final double PITCH = Units.degreesToRadians(355.256834);
      private static final double YAW = Units.degreesToRadians(15.619742);
    }

    private class FrontLeft {
      private static final double X = Units.inchesToMeters(-4.757613);
      private static final double Y = Units.inchesToMeters(7.938785);
      private static final double Z = Units.inchesToMeters(27.046870);
      private static final double ROLL = Units.degreesToRadians(1.26);
      private static final double PITCH = Units.degreesToRadians(355.256834);
      private static final double YAW = Units.degreesToRadians(344.380258);
    }

    private class RearRight {
      private static final double X = Units.inchesToMeters(-13.852572);
      private static final double Y = Units.inchesToMeters(-9.047180);
      private static final double Z = Units.inchesToMeters(17.891914);
      private static final double ROLL = Units.degreesToRadians(0.0); // 352.904
      private static final double PITCH = Units.degreesToRadians(340.0); // 288.882
      private static final double YAW =
          Units.degreesToRadians(200); // 190 TODO: verify swapped yaws
    }

    private class RearLeft {
      private static final double X = Units.inchesToMeters(-13.846460);
      private static final double Y = Units.inchesToMeters(9.052008);
      private static final double Z = Units.inchesToMeters(17.903158);
      private static final double ROLL = Units.degreesToRadians(0.0); // 7.096
      private static final double PITCH = Units.degreesToRadians(340.0); // 288.882
      private static final double YAW = Units.degreesToRadians(160.0); // 170
    }

    // initializes cameras for use in VisionSubsystem
    public static enum VisionCamera {
      FRONT_RIGHT_CAM(
          "frontRightCam",
          new Transform3d(
              new Translation3d(FrontRight.X, FrontRight.Y, FrontRight.Z),
              new Rotation3d(FrontRight.ROLL, FrontRight.PITCH, FrontRight.YAW))),

      FRONT_LEFT_CAM(
          "frontLeftCam",
          new Transform3d(
              new Translation3d(FrontLeft.X, FrontLeft.Y, FrontLeft.Z),
              new Rotation3d(FrontLeft.ROLL, FrontLeft.PITCH, FrontLeft.YAW))),

      REAR_RIGHT_CAM(
          "rearRightCam",
          new Transform3d(
              new Translation3d(RearRight.X, RearRight.Y, RearRight.Z),
              new Rotation3d(RearRight.ROLL, RearRight.PITCH, RearRight.YAW))),

      REAR_LEFT_CAM(
          "rearLeftCam",
          new Transform3d(
              new Translation3d(RearLeft.X, RearLeft.Y, RearLeft.Z),
              new Rotation3d(RearLeft.ROLL, RearLeft.PITCH, RearLeft.YAW)));

      private String loggingName;
      private Transform3d cameraTransform;

      VisionCamera(String name, Transform3d transform) {
        loggingName = name;
        cameraTransform = transform;
      }

      public String getLoggingName() {
        return loggingName;
      }

      public Transform3d getCameraTransform() {
        return cameraTransform;
      }
    }

    public static final CameraSelectionMethod CAMERA_SELECTION_METHOD = CameraSelectionMethod.MIN;

    public static int MAX_JITTER_MEASUREMENTS = 16;

    public static enum CameraSelectionMethod {
      MIN(),
      AVG(),
      MAX(),
      POSE_AMBIGUITY(),
      JITTER();
    }
  }

  public static class FuelGaugeDetection {
    public static final int BALLS_TO_AVG = 3;
    public static final int MAX_FUEL_GAUGE_MEASUREMENTS = 33;
    public static final double MAX_DETECTABLE_FUEL_AREA_PERCENTAGE = 60.00;
    public static final double REALISTIC_MAX_DETECTABLE_AREA_PERCENTAGE = 15.00;

    public static final double FUEL_GAUGE_X = Units.inchesToMeters(-3.454827);
    public static final double FUEL_GAUGE_Y = Units.inchesToMeters(-7.056897);
    public static final double FUEL_GAUGE_Z = Units.inchesToMeters(25.105416);
    public static final double FUEL_GAUGE_ROLL = Units.degreesToRadians(286.894287);
    public static final double FUEL_GAUGE_PITCH = Units.degreesToRadians(55.646896);
    public static final double FUEL_GAUGE_YAW = Units.degreesToRadians(23.957651);

    public static enum FuelGaugeCamera {
      FUEL_GAUGE_CAM(
          "fuelGaugeCam",
          new Transform3d(
              new Translation3d(FUEL_GAUGE_X, FUEL_GAUGE_Y, FUEL_GAUGE_Z),
              new Rotation3d(FUEL_GAUGE_ROLL, FUEL_GAUGE_PITCH, FUEL_GAUGE_YAW)));

      private String loggingName;
      private Transform3d cameraTransform;

      FuelGaugeCamera(String name, Transform3d transform) {
        loggingName = name;
        cameraTransform = transform;
      }

      public String getLoggingName() {
        return loggingName;
      }

      public Transform3d getCameraTransform() {
        return cameraTransform;
      }
    }

    public static enum GaugeCalculationType {
      RAW(),
      SMOOTHED(),
      MULTIPLE_BALLS(),
      SMOOTHED_MULTIPLE_BALLS();
    }

    public static enum FuelGauge { // LAST: 20, 50, 70, 100
      EMPTY(2.0, "#000000"),
      LOW(9.0, "#FF0000"),
      MEDIUM(12.0, "#FFFF00"),
      FULL(100.0, "#00FF00");

      private double threshold;
      private String color;

      FuelGauge(double threshold, String color) {
        this.threshold = threshold;
        this.color = color;
      }

      public double getThreshold() {
        return threshold;
      }

      public String getColor() {
        return color;
      }
    }
  }

  // public static final class Shooter {
  // public static final int WARMUP_1_ID = 18;
  // public static final int WARMUP_2_ID = 19;
  // public static final int WARMUP_3_ID = 24;

  // public static final double WHEEL_TOLERANCE_RPS = 2.0; // tolerance in
  // rotations per second

  // // TODO: Tune
  // public static final double kP = 0.5;
  // public static final double kI = 0.0;
  // public static final double kD = 0.0;
  // public static final double kV = 0.12;
  // public static final double kA = 0.0;

  // public static final double STATOR_CURRENT_LIMIT = 30.0;
  // public static final double SUPPLY_CURRENT_LIMIT = 30.0;

  // public static final double MOTOR_ROTS_PER_WHEEL_ROT = 1.25;
  // public static final double WHEEL_ROTS_PER_MOTOR_ROT = 1.0 /
  // MOTOR_ROTS_PER_WHEEL_ROT;
  // public static final double SHOOTER_WHEEL_DIAMETER = 3.0;
  // public static final double SHOOT_FOR_AUTO = 66.7; //104.72 / 2.0;

  // public static final Pose3d OFFSET_FROM_ROBOT_CENTER = new Pose3d();

  // public static final double SHOOTER_ANGLE_FROM_HORIZONTAL_DEGREES = 75;

  // public static final boolean SHOOTS_BACKWARDS = false;

  // public static final double ANGULAR_TOLERANCE_FOR_AUTO_AIM_RAD = .1;

  // public static final int TARGETING_CALCULATION_PRECISION = 5;

  // public static final double MIN_DIST_FT = 4d;
  // public static final double MAX_DIST_FT = 8d;

  // public static final class Simulation {
  // public static final double SHOOTER_SIM_MOI_KG_M2 = 0.0015;
  // }
  // }
  public static final class Shooter {

    public static final boolean INTERMAP_TESTING = false;

    public static final int WARMUP_1_ID = 35; // TODO
    public static final int WARMUP_2_ID = 34; // TODO
    public static final int WARMUP_3_ID = 33; // TODO

    public static final double KP = 0.8; // TODO
    public static final double KI = 0.0; // TODO
    public static final double KD = 0.0; // TODO
    public static final double KV = 0.124; // TODO
    public static final double KA = 0.0; // TODO
    public static final double STATOR_CURRENT_LIMIT = 120.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;

    public static final double MOTOR_ROTS_PER_WHEEL_ROT = 1.25;
    public static final double WHEEL_ROTS_PER_MOTOR_ROT = 1.0 / MOTOR_ROTS_PER_WHEEL_ROT;
    public static final double SHOOTER_WHEEL_DIAMETER = 3.0;
    public static final double SHOOT_FOR_AUTO = 67.0;

    public static final Pose3d OFFSET_FROM_ROBOT_CENTER = new Pose3d();

    public static final double SHOOTER_ANGLE_FROM_HORIZONTAL_DEGREES = 75;

    public static final boolean SHOOTS_BACKWARDS = false;

    public static final double ANGULAR_TOLERANCE_FOR_AUTO_AIM_RAD = .1;

    public static final int TARGETING_CALCULATION_PRECISION = 5;

    public static final double MIN_DIST_FT = 4d;
    public static final double MAX_DIST_FT = 8d;

    public static final double SHOOTER_SIM_MOI_KG_M2 = 0.0015;

    public static final double HUB_EDGE_TO_HUB_CENTER_INCHES = 20d;
    public static final double ROBOT_FRONT_EDGE_TO_SHOOTER = 27d;
    public static final double ROBOT_FRONT_EDGE_TO_ROBOT_CENTER = 13.75d;

    public static final InterpolatingDoubleTreeMap SHOOTER_WHEEL_RPS_FOR_DISTANCE_METERS =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap
        TOF_FOR_DISTANCE_METERS_CENTER_TO_CENTER_INTERMAP = new InterpolatingDoubleTreeMap();

    static {
      UPDATE_INTERMAPS();
    }

    public static void UPDATE_INTERMAPS() {
      SHOOTER_WHEEL_RPS_FOR_DISTANCE_METERS.clear();

      final double offset = 1.0429875;

      SHOOTER_WHEEL_RPS_FOR_DISTANCE_METERS.put(0.2111 + offset, 45.2000038381);
      SHOOTER_WHEEL_RPS_FOR_DISTANCE_METERS.put(0.6108 + offset, 46.4732433828);
      SHOOTER_WHEEL_RPS_FOR_DISTANCE_METERS.put(1.0478 + offset, 50.9295817894);
      SHOOTER_WHEEL_RPS_FOR_DISTANCE_METERS.put(1.4097 + offset, 52.8394411065);
      SHOOTER_WHEEL_RPS_FOR_DISTANCE_METERS.put(1.7971 + offset, 55.385920196);
      SHOOTER_WHEEL_RPS_FOR_DISTANCE_METERS.put(2.1336 + offset, 57.2957795131);
      SHOOTER_WHEEL_RPS_FOR_DISTANCE_METERS.put(3.6576 + offset, 61.1154981473);

      double VIDEO_SECONDS_TO_REGULAR_SECONDS = 1d / 8d;
      TOF_FOR_DISTANCE_METERS_CENTER_TO_CENTER_INTERMAP.clear();
      TOF_FOR_DISTANCE_METERS_CENTER_TO_CENTER_INTERMAP.put(0d, 0d);
      TOF_FOR_DISTANCE_METERS_CENTER_TO_CENTER_INTERMAP.put(
          Units.inchesToMeters(
              13d + HUB_EDGE_TO_HUB_CENTER_INCHES + ROBOT_FRONT_EDGE_TO_ROBOT_CENTER),
          10d * VIDEO_SECONDS_TO_REGULAR_SECONDS);
      TOF_FOR_DISTANCE_METERS_CENTER_TO_CENTER_INTERMAP.put(
          Units.inchesToMeters(
              50d + HUB_EDGE_TO_HUB_CENTER_INCHES + ROBOT_FRONT_EDGE_TO_ROBOT_CENTER),
          13d * VIDEO_SECONDS_TO_REGULAR_SECONDS);
      TOF_FOR_DISTANCE_METERS_CENTER_TO_CENTER_INTERMAP.put(
          Units.inchesToMeters(
              70d + 5d / 8d + HUB_EDGE_TO_HUB_CENTER_INCHES + ROBOT_FRONT_EDGE_TO_ROBOT_CENTER),
          13d * VIDEO_SECONDS_TO_REGULAR_SECONDS);
      TOF_FOR_DISTANCE_METERS_CENTER_TO_CENTER_INTERMAP.put(
          Units.inchesToMeters(
              93d + 1d / 8d + HUB_EDGE_TO_HUB_CENTER_INCHES + ROBOT_FRONT_EDGE_TO_ROBOT_CENTER),
          14d * VIDEO_SECONDS_TO_REGULAR_SECONDS);
      TOF_FOR_DISTANCE_METERS_CENTER_TO_CENTER_INTERMAP.put(
          Units.inchesToMeters(
              102d + 5d / 8d + HUB_EDGE_TO_HUB_CENTER_INCHES + ROBOT_FRONT_EDGE_TO_ROBOT_CENTER),
          15d * VIDEO_SECONDS_TO_REGULAR_SECONDS);
    }
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

    public static Pose2d BLUE_HUB_2D =
        new Pose2d(4.621390342712402, 4.032095909118652, new Rotation2d());
    public static Pose2d RED_HUB_2D =
        new Pose2d(11.917659759521484, 4.032095909118652, new Rotation2d());

    public static Pose2d RED_TOWER_R =
        new Pose2d(14.871597290039062, 4.749175071716309, new Rotation2d(0));
    public static Pose2d RED_TOWER_L =
        new Pose2d(14.871597290039062, 3.892498254776001, new Rotation2d(0));
    public static Pose2d BLUE_TOWER_R =
        new Pose2d(1.6428194046020508, 3.320095539093017, new Rotation2d(Math.PI));
    public static Pose2d BLUE_TOWER_L =
        new Pose2d(1.6428194046020508, 4.1721110343933105, new Rotation2d(Math.PI));
  }
}
