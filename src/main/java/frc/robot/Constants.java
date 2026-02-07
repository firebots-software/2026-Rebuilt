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
  public static final boolean intakeOnRobot = true;
  public static final boolean visionOnRobot = false;
  public static final boolean shooterOnRobot = false;
  public static final boolean climberOnRobot = false;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class Intake {
    public static final class Arm {
      public static final MotorConstants ARM_MOTOR = new MotorConstants(34);

      public static final double MOTOR_ROTS_TO_ARM_ROTS = 1d / 77.8;
      public static final double MOTOR_ROTS_TO_ARM_DEGREES = MOTOR_ROTS_TO_ARM_ROTS * 360d;
      public static final double ARM_DEGREES_TO_MOTOR_ROTS = 1 / MOTOR_ROTS_TO_ARM_DEGREES;
      public static final double ENCODER_ROTS_TO_ARM_ROTS = 2.666;

      public static final double ARM_KV = 0.14;
      public static final double ARM_KP = 0.1;
      public static final double ARM_KI = 0;
      public static final double ARM_KD = 0;
      public static final double ARM_FEEDFORWARD = 0.1;

      public static final double ARM_STATOR_CURRENT_LIMIT = 40.0;

      public static final int ENCODER_PORT = 0;
      public static final double ENCODER_OFFSET = 0; // subject to change

      public static double ARM_TOLERANCE_DEGREES = 1;

      public static final double ARM_DEGREES_UPPER_LIMIT = 95.0;
      public static final double ARM_POS_INITIAL = 0;
      public static final double ARM_POS_RETRACTED = 90.0;
      public static final double ARM_POS_EXTENDED = 15.0;
      public static final double ARM_POS_IDLE = 45.0; // subject to change
    }

    public static final MotorConstants INTAKE_MOTOR = new MotorConstants(33);

    public static final double MOTOR_ROTS_TO_INTAKE_ROTS = 1d / 2.6667;
    public static final double ENCODER_ROTS_TO_INTAKE_ROTS = 2.666;
    // ( 3" diameter roller wheels / 12" ) * pi to calculate circumference of the wheel in feet
    // wheel circumference can be used to convert from intake rotations/sec -> feet/sec
    public static final double INTAKE_ROTS_PER_SEC_TO_FEET_PER_SEC = (3 / 12) * Math.PI;

    public static final double INTAKE_KV = 0.14;
    public static final double INTAKE_KP = 0.1;
    public static final double INTAKE_KI = 0;
    public static final double INTAKE_KD = 0;
    public static final double INTAKE_FEEDFORWARD = 0.1;

    public static final double INTAKE_SUPPLY_CURRENT_LIMIT = 30.0;
    public static final double INTAKE_STATOR_CURRENT_LIMIT = 50.0;
    public static final double INTAKE_TARGET_SPEED =
        40.0 / MOTOR_ROTS_TO_INTAKE_ROTS; // subject to change
  }

  public static class MotorConstants {
    public int port;

    public MotorConstants(int port) {
      this.port = port;
    }
  }

  public static class Swerve {
    public static final SwerveType WHICH_SWERVE_ROBOT = SwerveType.COBRA;

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

    public static class MuscleUp {
      public static final double MUSCLE_UP_TOLERANCE = 0.1;

      public static final double MOTOR_ROTS_TO_ARM_ROTS = 1d / 250d;
      public static final double MOTOR_ROTS_PER_DEGREES_OF_ARM_ROT = MOTOR_ROTS_TO_ARM_ROTS * 360d;

      public static final double MUSCLE_UP_FORWARD = 0; // TODO: get vals
      public static final double MUSCLE_UP_BACK = 0; // TODO: get vals

      public static final int MOTOR_PORT = -1; // TODO: get vals

      public static final int ENCODER_PORT = -1; // TODO: get vals
      public static final int ENCODER_ROTATIONS_TO_ARM_ROTATIONS = 0;
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
    }

    public static class PullUp {
      public static final double PULL_UP_TOLERANCE = .1;

      public static final double MOTOR_ROTS_TO_PULLEY_ROTS = 1d / 17d;
      public static final double PULLEY_BELT_LENGTH_M = 0; // TODO: get actual value
      public static final double MOTOR_ROTS_PER_METERS_OF_BELT_TRAVERSAL =
          MOTOR_ROTS_TO_PULLEY_ROTS * PULLEY_BELT_LENGTH_M;

      public static final double REACH_POS = 0; // TODO: get vals
      public static final double PULL_DOWN_POS = 0; // TODO: get vals

      public static final int MOTOR_PORT_L = -1; // TODO: get vals
      public static final int MOTOR_PORT_R = -1; // TODO: get vals
    }
  }

  public static class Hopper {
    public static final double MOTOR_ROTS_TO_PULLEY_ROTS = .2d; // MRD
    private static final double PULLEY_LENGTH_MM = 220d * 5d; // 220 teeth, 5mm per
    private static final double PULLEY_LENGTH_M = PULLEY_LENGTH_MM / 1000d;
    public static final double MOTOR_ROTS_TO_METERS_OF_PULLEY_TRAVERSAL =
        MOTOR_ROTS_TO_PULLEY_ROTS * PULLEY_LENGTH_M;

    public static final double TARGET_PULLEY_SPEED_FT_PER_SEC = 6d;
    public static final double TARGET_PULLEY_SPEED_M_PER_SEC =
        Units.feetToMeters(TARGET_PULLEY_SPEED_FT_PER_SEC);

    public static final int MOTOR_PORT = 9; // TODO: put actual port

    public static final double kP = .07; // TODO: get actual vals
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kV = 0.12;

    public static final double HOPPER_STATOR_LIMIT = 30.0;
    public static final double HOPPER_SUPPLY_LIMIT = 30.0;

    public static final double TOLERANCE_MOTOR_ROTS_PER_SEC = .1;
  }

  public static class Vision {

    // initializes cameras for use in VisionSubsystem
    public static enum Cameras {
      RIGHT_CAM("rightCam"),
      LEFT_CAM("leftCam");

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

    // placeholder constants for now; will be updated once robot is delivered
    public static final double RIGHT_X = Units.inchesToMeters(8.867);
    public static final double RIGHT_Y = Units.inchesToMeters(-12.4787);
    public static final double RIGHT_Z = Units.inchesToMeters(6.158);
    public static final double RIGHT_ROLL = Units.degreesToRadians(0.0);
    public static final double RIGHT_PITCH = Units.degreesToRadians(-12.5);
    public static final double RIGHT_YAW = Units.degreesToRadians(40);

    public static final double LEFT_X = Units.inchesToMeters(8.867);
    public static final double LEFT_Y = Units.inchesToMeters(12.478);
    public static final double LEFT_Z = Units.inchesToMeters(6.158);
    public static final double LEFT_ROLL = Units.degreesToRadians(0.0);
    public static final double LEFT_PITCH = Units.degreesToRadians(-12.5);
    public static final double LEFT_YAW = Units.degreesToRadians(-40);

    // initializing Transform3d for use in future field visualization
    public static Transform3d getCameraTransform(Cameras camera) {
      switch (camera) {
        case RIGHT_CAM:
          return new Transform3d(
              new Translation3d(RIGHT_X, RIGHT_Y, RIGHT_Z),
              new Rotation3d(RIGHT_ROLL, RIGHT_PITCH, RIGHT_YAW));
        case LEFT_CAM:
          return new Transform3d(
              new Translation3d(LEFT_X, LEFT_Y, LEFT_Z),
              new Rotation3d(LEFT_ROLL, LEFT_PITCH, LEFT_YAW));
        default:
          throw new IllegalArgumentException("Unknown camera ID: " + camera);
      }
    }
  }

  public static final class Shooter {
    public static final MotorConstants warmUpMotor1 = new MotorConstants(35); // TODO
    public static final MotorConstants warmUpMotor2 = new MotorConstants(34); // TODO
    public static final MotorConstants warmUpMotor3 = new MotorConstants(32); // TODO

    public static final double SHOOTER_KP = 0.0; // TODO
    public static final double SHOOTER_KI = 0.0; // TODO
    public static final double SHOOTER_KD = 0.0; // TODO
    public static final double SHOOTER_KV = 0.0; // TODO
    public static final double SHOOTER_KA = 0.0; // TODO
    public static final double STATOR_CURRENT_LIMIT = 30.0;
    public static final double SUPPLY_CURRENT_LIMIT = 30.0;

    public static final double SHOOTER_WHEEL_GEAR_RATIO = 1.25;
    public static final double SHOOTER_WHEEL_DIAMETER = 3.0;
    public static final double SHOOT_FOR_AUTO = 104.72;

    public static final Pose3d OFFSET_FROM_ROBOT_CENTER = new Pose3d();

    public static final double SHOOTER_ANGLE_FROM_HORIZONTAL_DEGREES = 75;

    public static final boolean SHOOTS_BACKWARDS = false;

    public static final double ANGULAR_TOLERANCE_FOR_AUTO_AIM_RAD = .1;

    public static final int TARGETING_CALCULATION_PRECISION = 5;
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
