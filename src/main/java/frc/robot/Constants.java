// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

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

  public static class Landmarks {

    public static final double MIDLINE_X = 0;

  }

  public static final class Shooter {
    public static final MotorConstants motor1Constants = new MotorConstants(30);
    public static final MotorConstants motor2Constants = new MotorConstants(31);
    public static final MotorConstants preShooterConstants = new MotorConstants(32);

    public static final int ObjectDetectorPort = 1;
  }

  public static final class Arm {
    public static final double BUNDT_ANGLE = 20d;
    public static final double ARM_STATOR_CURRENT_LIMIT_AMPS = 20.0;
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

    // TODO: TUNE THESE CONSTANTS
    public static final double S0C_KP = 0.35;
    public static final double S0C_KI = 0.0; // optional, not recommended
    public static final double S0C_KD = 0.0; // optional, not recommended
    public static final double S0C_KS = 0.0;
    public static final double S0C_KG = 0.25;
    public static final double S0C_KA = 0.0; // kA not used
    public static final double S0C = 0.0;

    public static final double FEET_TO_METERS_CONVERSION_FACTOR = 0.3048;
    public static final double ABSOLUTE_ARM_CONVERSION_FACTOR = 42d / 18d;
    public static final double INTEGRATED_ABSOLUTE_CONVERSION_FACTOR = 55.9867;
    public static final double INTEGRATED_ARM_CONVERSION_FACTOR =
        ABSOLUTE_ARM_CONVERSION_FACTOR
            * INTEGRATED_ABSOLUTE_CONVERSION_FACTOR; // 130.63563333333335;

    public static final double MOTIONMAGIC_CRUISE_VELOCITY = 134.0 * INTEGRATED_ARM_CONVERSION_FACTOR / 360f; // MotionMagic Cruise Velocity in RPS of the arm
    public static final double MOTIONMAGIC_ACCEL = 200.0 * INTEGRATED_ARM_CONVERSION_FACTOR / 360f; // MotionMagic Acceleration in RPS^2 of the arm

    public static final double ABSOLUTE_ENCODER_HORIZONTAL = 0.6655; // 0.6547
    public static final double ABSOLUTE_HORIZONTAL_OFFSET = 0.05; // 0.05
    public static double ARM_INTERMAP_OFFSET = 0;
    // public static double ZERO_SPEAKER_OFFSET_METERS = 0.6;
    public static final InterpolatingDoubleTreeMap INTERMAP = new InterpolatingDoubleTreeMap();

    static {
      UPDATE_INTERMAP();
    }

    public static void UPDATE_INTERMAP() {
      // if (Constants.Pooer.SHOOTER == ShooterType.PETER) {
      //   UPDATE_INTERMAP_PETER();
      // } else {
        UPDATE_INTERMAP_PIPER();
      // }
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

  public static class MotorConstants {
    public int port;

    public MotorConstants(int port) {
      this.port = port;
    }
  }
}
