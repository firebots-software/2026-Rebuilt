package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Landmarks;
import frc.robot.util.LoggedTalonFX;
import frc.robot.util.Targeting;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class HoodedShooterSubsystem extends SubsystemBase {

  private final CommandSwerveDrivetrain drivetrain;
  private final BooleanSupplier redside;
  private final LoggedTalonFX warmup1, warmup2, warmup3, shooter, hood;
  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
  private final PositionVoltage m_positionRequest = new PositionVoltage(0);
  private double targetShooterSpeedRPS = 0;
  private double targetHoodPosition = 0;

  public HoodedShooterSubsystem(CommandSwerveDrivetrain drivetrain, BooleanSupplier redside) {
    this.drivetrain = drivetrain;
    this.redside = redside;
    CANBus canbus = Constants.Swerve.CAN_BUS;
    warmup1 = new LoggedTalonFX("ShooterWarmup1", Constants.Shooter.WARMUP_1_ID, canbus);
    warmup2 = new LoggedTalonFX("ShooterWarmup2", Constants.Shooter.WARMUP_2_ID, canbus);
    warmup3 = new LoggedTalonFX("ShooterWarmup3", Constants.Shooter.WARMUP_3_ID, canbus);
    shooter = warmup3;
    // TODO: fix hood id constant
    hood =
        new LoggedTalonFX("ShooterHood", Constants.Shooter.Hood.HOOD_ID, Constants.Swerve.CAN_BUS);

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
    VoltageConfigs voltageConfigs = new VoltageConfigs().withPeakReverseVoltage(0.0);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0 = s0c;
    config.CurrentLimits = clc;
    config.MotorOutput = motorOutputConfigs;
    config.Voltage = voltageConfigs;

    warmup1.getConfigurator().apply(config);
    warmup2.getConfigurator().apply(config);
    warmup3.getConfigurator().apply(config);

    Follower follower = new Follower(Constants.Shooter.WARMUP_3_ID, MotorAlignmentValue.Aligned);
    warmup1.setControl(follower);
    warmup2.setControl(follower);

    // TODO: Verify all this
    Slot0Configs hoodS0c =
        new Slot0Configs()
            .withKP(Constants.Shooter.Hood.KP)
            .withKI(Constants.Shooter.Hood.KI)
            .withKD(Constants.Shooter.Hood.KD)
            .withKV(Constants.Shooter.Hood.KV)
            .withKA(Constants.Shooter.Hood.KA);
    CurrentLimitsConfigs hoodClc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Shooter.Hood.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.Shooter.Hood.SUPPLY_CURRENT_LIMIT);
    MotorOutputConfigs hoodMotorOutputConfigs =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
    VoltageConfigs hoodVoltageConfigs = new VoltageConfigs().withPeakReverseVoltage(0.0);

    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    hoodConfig.Slot0 = hoodS0c;
    hoodConfig.CurrentLimits = hoodClc;
    hoodConfig.MotorOutput = hoodMotorOutputConfigs;
    hoodConfig.Voltage = hoodVoltageConfigs;

    hood.getConfigurator().apply(hoodConfig);

    DogLog.log("Subsystems/Shooter/Gains/kP", Constants.Shooter.KP);
    DogLog.log("Subsystems/Shooter/Gains/kI", Constants.Shooter.KI);
    DogLog.log("Subsystems/Shooter/Gains/kD", Constants.Shooter.KD);
    DogLog.log("Subsystems/Shooter/Gains/kV", Constants.Shooter.KV);
    DogLog.log("Subsystems/Shooter/Gains/kA", Constants.Shooter.KA);

    DogLog.log("Subsystems/Shooter/Hood/Gains/kP", Constants.Shooter.Hood.KP);
    DogLog.log("Subsystems/Shooter/Hood/Gains/kI", Constants.Shooter.Hood.KI);
    DogLog.log("Subsystems/Shooter/Hood/Gains/kD", Constants.Shooter.Hood.KD);
    DogLog.log("Subsystems/Shooter/Hood/Gains/kV", Constants.Shooter.Hood.KV);
    DogLog.log("Subsystems/Shooter/Hood/Gains/kA", Constants.Shooter.Hood.KA);
  }

  public void setHoodPosition(double hoodPosition) {
    targetHoodPosition = hoodPosition;
    hood.setControl(
        m_positionRequest.withPosition(
            hoodPosition * Constants.Shooter.Hood.MOTOR_ROTS_PER_DEGREE));
  }

  public void setShooterSpeedRPS(double shooterSpeedRPS) {
    targetShooterSpeedRPS = shooterSpeedRPS;
    shooter.setControl(
        m_velocityRequest.withVelocity(
            targetShooterSpeedRPS * Constants.Shooter.MOTOR_ROTS_PER_WHEEL_ROT));
  }

  public void stopShooter() {
    targetShooterSpeedRPS = 0;
    shooter.stopMotor();
  }

  public boolean isAtSpeed() {
    if (shooter.getCachedVelocityRps() == 0) {
      return false;
    }
    return Math.abs(targetShooterSpeedRPS - (getCurrentShooterWheelSpeedRPS()))
        <= Constants.Shooter.TOLERANCE_RPS;
  }

  public double getCurrentShooterWheelSpeedRPS() {
    return shooter.getCachedVelocityRps() * Constants.Shooter.WHEEL_ROTS_PER_MOTOR_ROT;
  }

  public double getTargetShooterWheelSpeedRPS() {
    return targetShooterSpeedRPS;
  }

  public double grabTargetShootingSpeed(double distanceToTarget) {
    double mappedSpeed =
        (Constants.Shooter.SHOOTER_WHEEL_RPS_FOR_DISTANCE_METERS.get(distanceToTarget)); // -0.4

    return mappedSpeed;
  }

  // Commands
  public Command shootAtSpeedCommand() {
    return runEnd(() -> setShooterSpeedRPS(67.0), this::stopShooter);
  }

  public Command shootAtSpeedCommand(double shooterSpeedRPS) {
    return runEnd(() -> setShooterSpeedRPS(shooterSpeedRPS), this::stopShooter);
  }

  public Command shootAtSpeedCommand(DoubleSupplier shooterSpeedRPS) {
    DogLog.log("Subsystems/Shooter/ShootingSpeedRN", shooterSpeedRPS.getAsDouble());
    return runEnd(() -> this.setShooterSpeedRPS(shooterSpeedRPS.getAsDouble()), this::stopShooter);
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
