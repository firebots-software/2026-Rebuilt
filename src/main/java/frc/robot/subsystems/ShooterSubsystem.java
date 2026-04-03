package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
  private final LoggedTalonFX warmup1, warmup2, warmup3, shooter, hood;
  private final CANcoder hoodEncoder;
  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
  private final PositionVoltage m_positionRequest = new PositionVoltage(0);
  private final VoltageOut m_voltageRequest = new VoltageOut(0);
  private double targetShooterSpeedRPS = 0;
  private double hoodTargetDeg = 0;

  public ShooterSubsystem(CommandSwerveDrivetrain drivetrain, BooleanSupplier redside) {
    this.drivetrain = drivetrain;
    this.redside = redside;
    CANBus canbus = Constants.Swerve.CAN_BUS;
    warmup1 = new LoggedTalonFX("ShooterWarmup1", Constants.Shooter.Rollers.WARMUP_1_ID, canbus);
    warmup2 = new LoggedTalonFX("ShooterWarmup2", Constants.Shooter.Rollers.WARMUP_2_ID, canbus);
    warmup3 = new LoggedTalonFX("ShooterWarmup3", Constants.Shooter.Rollers.WARMUP_3_ID, canbus);
    shooter = warmup3;

    // TODO: fix hood id constant
    hood =
        new LoggedTalonFX("ShooterHood", Constants.Shooter.Hood.HOOD_ID, Constants.Swerve.CAN_BUS);

    Slot0Configs rollersS0c =
        new Slot0Configs()
            .withKP(Constants.Shooter.Rollers.KP)
            .withKV(Constants.Shooter.Rollers.KV)
            .withKS(Constants.Shooter.Rollers.KS);

    CurrentLimitsConfigs rollersClc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Shooter.Rollers.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.Shooter.Rollers.SUPPLY_CURRENT_LIMIT);

    MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);

    VoltageConfigs voltageConfigs = new VoltageConfigs().withPeakReverseVoltage(0.0);

    TalonFXConfiguration rollersConfig = new TalonFXConfiguration();
    rollersConfig.Slot0 = rollersS0c;
    rollersConfig.CurrentLimits = rollersClc;
    rollersConfig.MotorOutput = motorOutputConfigs;
    rollersConfig.Voltage = voltageConfigs;

    warmup1.getConfigurator().apply(rollersConfig);
    warmup2.getConfigurator().apply(rollersConfig);
    warmup3.getConfigurator().apply(rollersConfig);

    Follower follower =
        new Follower(Constants.Shooter.Rollers.WARMUP_3_ID, MotorAlignmentValue.Aligned);
    warmup1.setControl(follower);
    warmup2.setControl(follower);

    hoodEncoder = new CANcoder(Constants.Shooter.Hood.ENCODER_PORT, Constants.Swerve.CAN_BUS);

    // TODO: Verify all this
    Slot0Configs hoodS0c =
        new Slot0Configs()
            .withKP(Constants.Shooter.Hood.KP)
            .withKV(Constants.Shooter.Hood.KV)
            .withKS(Constants.Shooter.Hood.KS)
            .withKG(Constants.Shooter.Hood.KG)
            .withKD(Constants.Shooter.Hood.KD);

    CurrentLimitsConfigs hoodClc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Shooter.Hood.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.Shooter.Hood.SUPPLY_CURRENT_LIMIT);

    // TODO: Verify these configurations
    MotorOutputConfigs hoodMotorOutputConfigs =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

    FeedbackConfigs hoodFeedbackConfigs =
        new FeedbackConfigs()
            .withFeedbackRemoteSensorID(hoodEncoder.getDeviceID())
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            .withSensorToMechanismRatio(Constants.Shooter.Hood.ENCODER_ROTS_PER_ARM_ROT)
            .withRotorToSensorRatio(Constants.Shooter.Hood.MOTOR_ROTS_PER_ENCODER_ROT);

    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    hoodConfig.Slot0 = hoodS0c;
    hoodConfig.CurrentLimits = hoodClc;
    hoodConfig.MotorOutput = hoodMotorOutputConfigs;
    hoodConfig.Feedback = hoodFeedbackConfigs;

    hood.getConfigurator().apply(hoodConfig);

    MagnetSensorConfigs hoodCANcoderConfig =
        new CANcoderConfiguration()
            .MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(1))
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                .withMagnetOffset(Rotations.of(Constants.Shooter.Hood.ENCODER_OFFSET));

    hoodEncoder.getConfigurator().apply(hoodCANcoderConfig);

    DogLog.log("Subsystems/Shooter/Gains/kP", Constants.Shooter.Rollers.KP);
    DogLog.log("Subsystems/Shooter/Gains/kV", Constants.Shooter.Rollers.KV);

    DogLog.log("Subsystems/Shooter/Hood/Gains/kP", Constants.Shooter.Hood.KP);
    DogLog.log("Subsystems/Shooter/Hood/Gains/kS", Constants.Shooter.Hood.KS);
    DogLog.log("Subsystems/Shooter/Hood/Gains/kG", Constants.Shooter.Hood.KG);
    DogLog.log("Subsystems/Shooter/Hood/Gains/kV", Constants.Shooter.Hood.KV);
    DogLog.log("Subsystems/Shooter/Hood/Gains/kD", Constants.Shooter.Hood.KD);
  }

  public void setHoodPosition(double degrees) {
    hoodTargetDeg = degrees;
    hood.setControl(
        m_positionRequest.withPosition(
            MathUtil.clamp(
                degrees / 360.0,
                Constants.Shooter.Hood.MIN_HOOD_POSITION,
                Constants.Shooter.Hood.MAX_HOOD_POSITION)));
  }

  public Rotation2d getHoodPosition() {
    return new Rotation2d(Units.rotationsToRadians(hood.getCachedPositionRotations()));
  }

  public void stopHood() {
    hoodTargetDeg = hood.getCachedPositionRotations() * 360.0;
    setHoodPosition(hoodTargetDeg);
  }

  public boolean hoodAtTarget() {
    return Math.abs(
            (hood.getCachedPositionRotations() * 360.0)
                - hoodTargetDeg)
        <= Constants.Shooter.Hood.HOOD_TOLERANCE;
  }

  public double getHoodCancoderPositionRaw() {
    return hoodEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public Rotation2d getHoodUnfusedPosition() {
    return new Rotation2d(
        Units.rotationsToRadians(
            getHoodCancoderPositionRaw() * Constants.Shooter.Hood.HOOD_ROTS_PER_CANCODER_ROT));
  }

  public void setShooterSpeedRPS(double shooterSpeedRPS) {
    targetShooterSpeedRPS = shooterSpeedRPS;
    shooter.setControl(
        m_velocityRequest.withVelocity(
            targetShooterSpeedRPS * Constants.Shooter.Rollers.MOTOR_ROTS_PER_WHEEL_ROT));
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
        <= Constants.Shooter.Rollers.TOLERANCE_RPS;
  }

  public double getCurrentShooterWheelSpeedRPS() {
    return shooter.getCachedVelocityRps() * Constants.Shooter.Rollers.WHEEL_ROTS_PER_MOTOR_ROT;
  }

  public double getTargetShooterWheelSpeedRPS() {
    return targetShooterSpeedRPS;
  }

  public double getTargetShootingSpeed(double distanceToTarget) {
    double mappedSpeed =
        Constants.Shooter.Rollers.SHOOTER_WHEEL_RPS_FOR_DISTANCE_METERS.get(
            distanceToTarget); // -0.4

    return mappedSpeed;
  }

  public double getTargetHoodAngle(double distanceToTarget) {
    double mappedAngle =
        Constants.Shooter.Hood.HOOD_ANGLE_FOR_DISTANCE_METERS_CENTER_TO_CENTER_INTERMAP.get(
            distanceToTarget);

    return mappedAngle;
  }

  // Commands
  public Command shootAtSpeedCommand() {
    return runEnd(() -> setShooterSpeedRPS(67.0), this::stopShooter);
  }

  public Command shootAtSpeedHoodCommand(double shooterSpeedRPS, double hoodAngle) {
    return runEnd(() -> setShooterSpeedRPS(shooterSpeedRPS), this::stopShooter)
        .alongWith(run(() -> setHoodPosition(hoodAngle)));
  }

  public Command shootAtSpeedCommand(double shooterSpeedRPS) {
    return runEnd(() -> setShooterSpeedRPS(shooterSpeedRPS), this::stopShooter);
  }

  public Command shootAtSpeedHoodCommand(DoubleSupplier shooterSpeedRPS, DoubleSupplier hoodAngle) {
    return runEnd(() -> setShooterSpeedRPS(shooterSpeedRPS.getAsDouble()), this::stopShooter)
        .alongWith(run(() -> setHoodPosition(hoodAngle.getAsDouble())));
  }

  public Command shootAtSpeedCommand(DoubleSupplier shooterSpeedRPS) {
    DogLog.log("Subsystems/Shooter/ShootingSpeedRN", shooterSpeedRPS.getAsDouble());
    return runEnd(() -> this.setShooterSpeedRPS(shooterSpeedRPS.getAsDouble()), this::stopShooter);
  }

  public void moveHoodWithVoltage() {
    hood.setControl(m_voltageRequest.withOutput(Constants.Shooter.Hood.ZERO_VOLTAGE));
  }

  public void resetHoodPositionToZero() {
    hood.setPosition(0);
  }

  public void reduceHoodCurrentLimits() {
    hood.updateCurrentLimits(
        Constants.Shooter.Hood.ZERO_STATOR_CURRENT_LIMIT,
        Constants.Shooter.Hood.ZERO_SUPPLY_CURRENT_LIMIT);
  }

  public void resetHoodCurrentLimits() {
    hood.updateCurrentLimits(
        Constants.Shooter.Hood.STATOR_CURRENT_LIMIT, Constants.Shooter.Hood.SUPPLY_CURRENT_LIMIT);
  }

  public boolean checkHoodCurrent() {
    double supply = Math.abs(hood.getSupplyCurrent().getValue().magnitude());
    double stator = Math.abs(hood.getStatorCurrent().getValue().magnitude());

    return supply > Constants.Shooter.Hood.ZERO_MAX_SUPPLY
        && stator > Constants.Shooter.Hood.ZERO_MAX_STATOR;
  }

  @Override
  public void periodic() {
    DogLog.log("Subsystems/Shooter/TargetWheelSpeed (rps)", getTargetShooterWheelSpeedRPS());
    DogLog.log("Subsystems/Shooter/AtTargetSpeed", isAtSpeed());
    DogLog.log("Subsystems/Shooter/CurrentSpeed (rps)", getCurrentShooterWheelSpeedRPS());

    Pose3d target = redside.getAsBoolean() ? Landmarks.RED_HUB : Landmarks.BLUE_HUB;

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
  }
}
