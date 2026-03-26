// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Landmarks;
import frc.robot.util.LoggedTalonFX;
import frc.robot.util.Targeting;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ShooterSubsystem extends SubsystemBase {
  private final CommandSwerveDrivetrain drivetrain;
  private final BooleanSupplier redside;

  private final LoggedTalonFX warmUpMotor1, warmUpMotor2, warmUpMotor3, shooter;
  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
  private double targetShooterWheelRPS = 0;
  private static final double TOLERANCE_RPS = 2.0; // tolerance in rotations per second

  public ShooterSubsystem(CommandSwerveDrivetrain drivetrain, BooleanSupplier redside) {
    this.drivetrain = drivetrain;
    this.redside = redside;

    warmUpMotor1 =
        new LoggedTalonFX(
            "ShooterWarmUp1", Constants.Shooter.WARMUP_1_ID, Constants.Swerve.CAN_BUS);
    warmUpMotor2 =
        new LoggedTalonFX(
            "ShooterWarmUp2", Constants.Shooter.WARMUP_2_ID, Constants.Swerve.CAN_BUS);
    warmUpMotor3 =
        new LoggedTalonFX(
            "ShooterWarmUp3", Constants.Shooter.WARMUP_3_ID, Constants.Swerve.CAN_BUS);
    shooter = warmUpMotor3;

    Slot0Configs s0c =
        new Slot0Configs()
            .withKP(Constants.Shooter.KP)
            .withKI(Constants.Shooter.KI)
            .withKD(Constants.Shooter.KD)
            .withKV(Constants.Shooter.KV)
            .withKA(Constants.Shooter.KA);

    CurrentLimitsConfigs clc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Shooter.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.Shooter.SUPPLY_CURRENT_LIMIT);

    MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);

    VoltageConfigs vConfigs = new VoltageConfigs().withPeakReverseVoltage(0.0);

    // Apply full TalonFXConfiguration to ensure factory defaults
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0 = s0c;
    config.CurrentLimits = clc;
    config.MotorOutput = motorOutputConfigs;
    config.Voltage = vConfigs;

    TalonFXConfigurator m1config = warmUpMotor1.getConfigurator();
    TalonFXConfigurator m2config = warmUpMotor2.getConfigurator();
    TalonFXConfigurator m3config = warmUpMotor3.getConfigurator();

    m1config.apply(config);
    m2config.apply(config);
    m3config.apply(config);

    // Set motors 1 and 2 to follow motor 3 (the leader)
    Follower follower = new Follower(Constants.Shooter.WARMUP_3_ID, MotorAlignmentValue.Aligned);
    warmUpMotor1.setControl(follower);
    warmUpMotor2.setControl(follower);

    DogLog.log("Subsystems/Shooter/Gains/kP", Constants.Shooter.KP);
    DogLog.log("Subsystems/Shooter/Gains/kI", Constants.Shooter.KI);
    DogLog.log("Subsystems/Shooter/Gains/kD", Constants.Shooter.KD);
    DogLog.log("Subsystems/Shooter/Gains/kV", Constants.Shooter.KV);
    DogLog.log("Subsystems/Shooter/Gains/kA", Constants.Shooter.KA);
  }

  public void setShooterWheelRPS(double shooterWheelSpeedRPS) {
    targetShooterWheelRPS = shooterWheelSpeedRPS;
    shooter.setControl(
        m_velocityRequest.withVelocity(
            targetShooterWheelRPS * Constants.Shooter.MOTOR_ROTS_PER_WHEEL_ROT));
  }

  public void stopShooter() {
    targetShooterWheelRPS = 0;
    shooter.stopMotor();
  }

  public boolean isAtSpeed() {
    if (shooter.getCachedVelocityRps() == 0) {
      return false;
    }
    return Math.abs(targetShooterWheelRPS - (getCurrentShooterWheelSpeedRPS())) <= TOLERANCE_RPS;
  }

  public double getCurrentShooterWheelSpeedRPS() {
    return shooter.getCachedVelocityRps() * Constants.Shooter.WHEEL_ROTS_PER_MOTOR_ROT;
  }

  public double getTargetShooterWheelSpeedRPS() {
    return targetShooterWheelRPS;
  }

  public double grabTargetShootingSpeed(double distanceToTarget) {
    double mappedSpeed = (Constants.Shooter.SHOOTER_WHEEL_RPS_FOR_DISTANCE_METERS.get(distanceToTarget)); // -0.4

    return mappedSpeed;
  }

  // Commands
  public Command shootAtSpeedCommand() {
    return runEnd(() -> setShooterWheelRPS(67.0), this::stopShooter);
  }

  public Command shootAtSpeedCommand(double shooterWheelSpeedRPS) {
    return runEnd(() -> setShooterWheelRPS(shooterWheelSpeedRPS), this::stopShooter);
  }

  public Command shootAtSpeedCommand(DoubleSupplier shooterWheelSpeedRPS) {
    DogLog.log("Subsystems/Shooter/ShootingSpeedRN", shooterWheelSpeedRPS.getAsDouble());
    return runEnd(
        () -> this.setShooterWheelRPS(shooterWheelSpeedRPS.getAsDouble()), this::stopShooter);
  }

  @Override
  public void periodic() {
    DogLog.log("Subsystems/Shooter/TargetWheelSpeed (rps)", getTargetShooterWheelSpeedRPS());
    DogLog.log("Subsystems/Shooter/AtTargetSpeed", isAtSpeed());
    DogLog.log("Subsystems/Shooter/CurrentSpeed (rps)", getCurrentShooterWheelSpeedRPS());

    Pose3d target = redside.getAsBoolean() ? Landmarks.RED_HUB : Landmarks.BLUE_HUB;

    // TODO: Cache this value
    DogLog.log(
        "Subsystems/Shooter/Targeting/TargetPlusLead",
        new Pose2d(
            Targeting.positionToTarget(
                    target, drivetrain, Constants.Shooter.TARGETING_CALCULATION_PRECISION)
                .x,
            Targeting.positionToTarget(
                    target, drivetrain, Constants.Shooter.TARGETING_CALCULATION_PRECISION)
                .y,
            new Rotation2d()));
    DogLog.log(
        "Subsystems/Shooter/Targeting/ShootingSpeed",
        Targeting.shootingSpeed(
            target, drivetrain, Constants.Shooter.TARGETING_CALCULATION_PRECISION));
    DogLog.log(
        "Subsystems/Shooter/Targeting/DistanceMeters", Targeting.distMeters(drivetrain, target));
    DogLog.log(
        "Subsystems/Shooter/Targeting/TargetAngle", Targeting.targetAngle(target, drivetrain));
    DogLog.log(
        "Subsystems/Shooter/Targeting/IsPointing", Targeting.pointingAtTarget(target, drivetrain));

    DogLog.log(
        "Subsystems/Shooter/Targeting/TimeOfFlight",
        Targeting.targetingInfo(
                target, drivetrain, Constants.Shooter.TARGETING_CALCULATION_PRECISION)
            .getToF());
    // DogLog.log("Subsystems/Shooter/CurrentSpeed (rps)",
    // shooter.getVelocity().getValueAsDouble());
  }
}
